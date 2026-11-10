/**
 * In-memory ROSLIB double for Jest. Records every Topic and Service that is
 * constructed, every publish and every service call; a test answers service
 * calls by registering a handler for the service name.
 */
export const installMockRoslib = () => {
  const registry = { topics: [], services: [], handlers: {} };

  class Topic {
    constructor(opts) {
      this.opts = opts;
      this.name = opts.name;
      this.messageType = opts.messageType;
      this.published = [];
      this.callbacks = [];
      this.advertised = false;
      registry.topics.push(this);
    }

    advertise() {
      this.advertised = true;
    }

    publish(message) {
      this.advertised = true;
      this.published.push(message);
    }

    subscribe(callback) {
      this.callbacks.push(callback);
    }

    unsubscribe(callback) {
      this.callbacks = callback ? this.callbacks.filter((cb) => cb !== callback) : [];
    }

    emit(message) {
      this.callbacks.forEach((cb) => cb(message));
    }
  }

  class Service {
    constructor(opts) {
      this.opts = opts;
      this.name = opts.name;
      this.serviceType = opts.serviceType;
      this.calls = [];
      registry.services.push(this);
    }

    callService(request, callback, errorCallback) {
      this.calls.push(request);
      const handler = registry.handlers[this.name];
      if (handler) handler(request, callback || (() => {}), errorCallback || (() => {}));
    }
  }

  class Message {
    constructor(values) {
      Object.assign(this, values);
    }
  }

  class ServiceRequest {
    constructor(values) {
      Object.assign(this, values);
    }
  }

  class Param {
    constructor(opts) {
      this.opts = opts;
    }

    get() {}

    set() {}
  }

  class Ros {
    constructor() {
      this.listeners = {};
    }

    on(event, fn) {
      this.listeners[event] = fn;
    }

    off(event) {
      delete this.listeners[event];
    }

    connect() {}

    close() {}

    getTopics(cb) {
      if (cb) cb({ topics: [], types: [] });
    }

    getNodes(cb) {
      if (cb) cb([]);
    }

    getServices(cb) {
      if (cb) cb([]);
    }

    getParams(cb) {
      if (cb) cb([]);
    }

    getTopicType(_topic, cb) {
      if (cb) cb("");
    }

    getServiceType(_service, cb) {
      if (cb) cb("");
    }
  }

  window.ROSLIB = { Topic, Service, Message, ServiceRequest, Param, Ros };

  return {
    registry,
    topicsNamed: (name) => registry.topics.filter((t) => t.name === name),
    servicesNamed: (name) => registry.services.filter((s) => s.name === name),
    callsTo: (name) =>
      registry.services.filter((s) => s.name === name).flatMap((s) => s.calls),
    onService: (name, handler) => {
      registry.handlers[name] = handler;
    },
  };
};

export const uninstallMockRoslib = () => {
  delete window.ROSLIB;
};
