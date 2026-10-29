"""HTTP API and static web UI for the feature manager."""

import asyncio
import json
import logging
from pathlib import Path
import threading
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .controller import FeatureController

LOG = logging.getLogger('feature_manager.web')

SSE_HEADERS = {'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'}
KEEPALIVE_S = 15.0


def _sse(data, event: str = None) -> str:
    head = f'event: {event}\n' if event else ''
    return f'{head}data: {json.dumps(data)}\n\n'


class UncachedStaticFiles(StaticFiles):
    """Make browsers revalidate (cheap via ETag), so an update never mixes old and new files."""

    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers['Cache-Control'] = 'no-cache'
        return response


class Goal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = 'map'


def create_app(controller: FeatureController, web_dir: Path, nav=None) -> FastAPI:
    """`nav` is the NavGoalClient of the ROS node, None when running without ROS."""
    app = FastAPI(title='Feature Manager', docs_url='/api/docs', redoc_url=None)

    def feature_or_404(feature_id: str):
        feature = controller.config.find(feature_id)
        if feature is None:
            raise HTTPException(404, f'Unknown feature: {feature_id}')
        return feature

    @app.get('/api/features')
    def list_features():
        return {
            'features': [f.to_dict() for f in controller.config.features.values()],
            'status': controller.snapshot(),
            'viewer': controller.config.viewer,
            'navigation': nav.snapshot() if nav else None,
        }

    @app.get('/api/status')
    def status():
        return controller.snapshot()

    @app.post('/api/features/{feature_id}/{action}', status_code=202)
    async def act(feature_id: str, action: str, request: Request):
        feature = feature_or_404(feature_id)
        body = {}
        if await request.body():
            try:
                body = await request.json()
            except ValueError:
                raise HTTPException(400, 'Body must be JSON')
        try:
            controller.request(feature.id, action, body.get('options'))
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        except RuntimeError as exc:
            raise HTTPException(409, str(exc))
        return {'ok': True, 'feature': feature.id, 'action': action}

    def nav_or_503():
        if nav is None:
            raise HTTPException(503, 'The feature manager runs without ROS and cannot send goals.')
        return nav

    @app.get('/api/navigation')
    def navigation():
        return nav_or_503().snapshot()

    @app.post('/api/navigation/goal', status_code=202)
    def navigation_goal(goal: Goal):
        try:
            nav_or_503().send(goal.x, goal.y, goal.yaw, goal.frame_id)
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        except RuntimeError as exc:
            raise HTTPException(409, str(exc))
        return {'ok': True}

    @app.post('/api/navigation/cancel', status_code=202)
    def navigation_cancel():
        try:
            nav_or_503().cancel()
        except RuntimeError as exc:
            raise HTTPException(409, str(exc))
        return {'ok': True}

    @app.get('/api/events')
    async def events(request: Request):
        async def stream():
            yield 'retry: 3000\n\n'
            last_version, last_nav, last_sent = None, None, 0.0
            while not await request.is_disconnected():
                now = time.monotonic()
                if nav and nav.version != last_nav:
                    last_nav, last_sent = nav.version, now
                    yield _sse(nav.snapshot(), 'navigation')
                if controller.version != last_version:
                    last_version, last_sent = controller.version, now
                    yield _sse(controller.snapshot(), 'status')
                elif now - last_sent > KEEPALIVE_S:
                    last_sent = now
                    yield ': keepalive\n\n'
                await asyncio.sleep(0.5)
        return StreamingResponse(stream(), media_type='text/event-stream', headers=SSE_HEADERS)

    @app.get('/api/features/{feature_id}/logs')
    def logs(feature_id: str, tail: int = 200):
        feature = feature_or_404(feature_id)
        return {'feature': feature.id, 'lines': controller.logs(feature.id, min(tail, 5000))}

    @app.get('/api/features/{feature_id}/logs/stream')
    async def logs_stream(feature_id: str, request: Request, tail: int = 200):
        feature = feature_or_404(feature_id)
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue(maxsize=5000)
        docker_stream, lines = await asyncio.to_thread(
            controller.follow_logs, feature.id, min(tail, 5000))

        def pump():
            # docker's log iterator blocks, so it runs in its own thread
            try:
                for line in lines:
                    loop.call_soon_threadsafe(queue.put_nowait, line)
            except Exception:  # noqa: B902 - stream closed or container removed
                pass
            finally:
                loop.call_soon_threadsafe(queue.put_nowait, None)

        threading.Thread(target=pump, name=f'logs-{feature.id}', daemon=True).start()

        async def stream():
            yield 'retry: 3000\n\n'
            yield _sse({'reset': True, 'exists': docker_stream is not None}, 'meta')
            try:
                while not await request.is_disconnected():
                    try:
                        line = await asyncio.wait_for(queue.get(), timeout=KEEPALIVE_S)
                    except asyncio.TimeoutError:
                        yield ': keepalive\n\n'
                        continue
                    if line is None:
                        yield _sse({'ended': True}, 'meta')
                        break
                    batch = [line]
                    while not queue.empty() and len(batch) < 500:
                        nxt = queue.get_nowait()
                        if nxt is None:
                            queue.put_nowait(None)
                            break
                        batch.append(nxt)
                    yield _sse(batch, 'lines')
            finally:
                if docker_stream is not None:
                    docker_stream.close()
        return StreamingResponse(stream(), media_type='text/event-stream', headers=SSE_HEADERS)

    @app.exception_handler(KeyError)
    async def key_error(_request, exc):
        return JSONResponse({'detail': str(exc)}, status_code=404)

    app.mount('/', UncachedStaticFiles(directory=str(web_dir), html=True), name='web')
    return app
