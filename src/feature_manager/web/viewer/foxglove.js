// Minimal client for the foxglove_bridge WebSocket protocol (foxglove.websocket.v1).
// Subscriptions are kept across reconnects; a topic is subscribed as soon as it is advertised.

import { MessageDecoder } from './ros2msg.js';

const OP_MESSAGE_DATA = 1;

export class FoxgloveClient extends EventTarget {
  constructor(url) {
    super();
    this.url = url;
    this.ws = null;
    this.state = 'closed';
    this.channels = new Map();      // channel id -> channel
    this.wanted = new Map();        // topic -> {handler, subId}
    this.subscriptions = new Map(); // subscription id -> {topic, decoder}
    this.decoders = new Map();      // channel id -> MessageDecoder
    this.nextSubId = 1;
    this.retryMs = 1000;
    this.stopped = true;
  }

  connect() {
    this.stopped = false;
    this.open();
  }

  close() {
    this.stopped = true;
    clearTimeout(this.retryTimer);
    if (this.ws) this.ws.close();
    this.ws = null;
    this.resetSession();
    this.setState('closed');
  }

  // Channel and subscription ids are only valid for one connection
  resetSession() {
    this.channels.clear();
    this.subscriptions.clear();
    this.decoders.clear();
    for (const want of this.wanted.values()) want.subId = null;
    this.dispatchEvent(new Event('channels'));
  }

  setState(state, detail = {}) {
    this.state = state;
    this.dispatchEvent(new CustomEvent('state', { detail: { state, ...detail } }));
  }

  open() {
    this.setState('connecting');
    let ws;
    try {
      ws = new WebSocket(this.url, ['foxglove.websocket.v1']);
    } catch (err) {
      this.setState('error', { error: String(err) });
      this.scheduleRetry();
      return;
    }
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onopen = () => {
      this.retryMs = 1000;
      this.setState('open');
    };
    ws.onmessage = (ev) => (typeof ev.data === 'string' ? this.onJson(JSON.parse(ev.data)) : this.onBinary(ev.data));
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.resetSession();
      if (!this.stopped) {
        this.setState('error', { error: `No connection to ${this.url}` });
        this.scheduleRetry();
      }
    };
  }

  scheduleRetry() {
    clearTimeout(this.retryTimer);
    this.retryTimer = setTimeout(() => !this.stopped && this.open(), this.retryMs);
    this.retryMs = Math.min(this.retryMs * 2, 10000);
  }

  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  onJson(msg) {
    if (msg.op === 'advertise') {
      for (const ch of msg.channels) this.channels.set(ch.id, ch);
      this.syncSubscriptions();
      this.dispatchEvent(new Event('channels'));
    } else if (msg.op === 'unadvertise') {
      for (const id of msg.channelIds) {
        this.channels.delete(id);
        this.decoders.delete(id);
        for (const [subId, sub] of this.subscriptions) {
          if (sub.channelId === id) {
            this.subscriptions.delete(subId);
            const want = this.wanted.get(sub.topic);
            if (want) want.subId = null;
          }
        }
      }
      this.syncSubscriptions();
      this.dispatchEvent(new Event('channels'));
    } else if (msg.op === 'status' && msg.level >= 2) {
      console.warn('foxglove_bridge:', msg.message);
    }
  }

  onBinary(data) {
    const view = new DataView(data);
    if (view.getUint8(0) !== OP_MESSAGE_DATA) return;
    const sub = this.subscriptions.get(view.getUint32(1, true));
    const want = sub && this.wanted.get(sub.topic);
    if (!want) return;
    let msg;
    try {
      msg = sub.decoder.decode(new Uint8Array(data, 13));
    } catch (err) {
      console.error(`Cannot decode ${sub.topic}:`, err);
      return;
    }
    want.handler(msg, sub.topic);
  }

  channelForTopic(topic) {
    for (const ch of this.channels.values()) if (ch.topic === topic) return ch;
    return null;
  }

  topicsWithSchema(schemaName) {
    const norm = (s) => s.replace('/msg/', '/');
    return [...this.channels.values()]
      .filter((ch) => norm(ch.schemaName) === norm(schemaName))
      .map((ch) => ch.topic)
      .sort();
  }

  subscribe(topic, handler) {
    const existing = this.wanted.get(topic);
    if (existing) {
      existing.handler = handler;
      return;
    }
    this.wanted.set(topic, { handler, subId: null });
    this.syncSubscriptions();
  }

  unsubscribe(topic) {
    const want = this.wanted.get(topic);
    if (!want) return;
    this.wanted.delete(topic);
    if (want.subId != null) {
      this.subscriptions.delete(want.subId);
      this.send({ op: 'unsubscribe', subscriptionIds: [want.subId] });
    }
  }

  syncSubscriptions() {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    const subscribe = [];
    for (const [topic, want] of this.wanted) {
      if (want.subId != null) continue;
      const ch = this.channelForTopic(topic);
      if (!ch) continue;
      if (ch.encoding !== 'cdr' || (ch.schemaEncoding && ch.schemaEncoding !== 'ros2msg')) {
        console.warn(`${topic}: unsupported encoding ${ch.encoding}/${ch.schemaEncoding}`);
        continue;
      }
      let decoder = this.decoders.get(ch.id);
      if (!decoder) {
        try {
          decoder = new MessageDecoder(ch.schemaName, ch.schema);
        } catch (err) {
          console.error(`Cannot parse schema of ${topic}:`, err);
          continue;
        }
        this.decoders.set(ch.id, decoder);
      }
      const subId = this.nextSubId++;
      want.subId = subId;
      this.subscriptions.set(subId, { topic, channelId: ch.id, decoder });
      subscribe.push({ id: subId, channelId: ch.id });
    }
    if (subscribe.length) this.send({ op: 'subscribe', subscriptions: subscribe });
  }
}
