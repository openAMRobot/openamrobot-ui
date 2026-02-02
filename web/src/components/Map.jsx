import React, {
  useCallback,
  useContext,
  useEffect,
  useRef,
  useState,
  forwardRef,
  useImperativeHandle,
} from "react";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants";

import MapButton from "../shared/ui/MapButton";

/**
 * NAV2D safety helpers
 */
const getNav2D = () => (typeof window !== "undefined" ? window.NAV2D : null);

const ensureNav2D = () => {
  if (typeof window === "undefined") return null;
  window.NAV2D = window.NAV2D || {};
  return window.NAV2D;
};

const getCanvas = () => {
  const nav2d = getNav2D();
  return nav2d?.canvas || null;
};

const getScene = () => {
  const canvas = getCanvas();
  return canvas?.scene || null;
};

const Map = forwardRef((props, ref) => {
  const ros = useContext(RosContext);

  const mapContainer = useRef(null);
  const mapItem = useRef(null);

  const viewerRef = useRef(null);

  const [canvasWidth, setCanvasWidth] = useState(undefined);
  const [canvasHeight, setCanvasHeight] = useState(undefined);

  // eslint-disable-next-line no-unused-vars
  const [mapPoints, updateMapPoints] = useState(0);

  // ROS topics refs (created lazily when ROSLIB exists)
  const mapTopicRef = useRef(null);
  const mapUpdateTopicRef = useRef(null);

  // Init retry interval (cleared automatically)
  const initRetryIntervalRef = useRef(null);

  useImperativeHandle(ref, () => ({
    getMapRef: () => mapItem.current,
  }));

  /**
   * Lazily create ROSLIB topics when ROSLIB is available.
   */
  useEffect(() => {
    if (typeof window === "undefined") return;
    if (!window.ROSLIB) return;
    if (!ros) return;

    if (!mapTopicRef.current) {
      mapTopicRef.current = new window.ROSLIB.Topic({
        ros,
        name: AppConfig.WP_REQ,
        messageType: "std_msgs/Empty",
      });
    }

    if (!mapUpdateTopicRef.current) {
      mapUpdateTopicRef.current = new window.ROSLIB.Topic({
        ros,
        name: "/map",
        messageType: "nav_msgs/OccupancyGrid",
      });
    }
  }, [ros]);

  /**
   * Create ROS2D.Viewer only if the container is empty.
   */
  const createCanvasContainer = useCallback(() => {
    if (typeof window === "undefined") return;

    const nav2d = ensureNav2D();
    if (!nav2d) return;

    if (!mapItem.current || !mapContainer.current) return;

    const mapElements = mapItem.current.childNodes;

    // If canvas already present, do nothing
    if (mapElements && mapElements.length >= 1) {
      if (!nav2d.canvas && viewerRef.current) {
        nav2d.canvas = viewerRef.current;
      }
      return;
    }

    const container = mapContainer.current;
    const containerWidth = container.offsetWidth;
    const containerHeight = container.offsetHeight;

    if (!containerWidth || !containerHeight) return;

    const maxCanvasHeight = containerHeight;

    let calculatedCanvasWidth = maxCanvasHeight / 0.7;
    let calculatedCanvasHeight;

    if (calculatedCanvasWidth > containerWidth) {
      calculatedCanvasWidth = containerWidth;
      calculatedCanvasHeight = calculatedCanvasWidth * 0.7;
    } else {
      calculatedCanvasHeight = maxCanvasHeight;
    }

    setCanvasWidth(calculatedCanvasWidth);
    setCanvasHeight(calculatedCanvasHeight);

    if (!window.ROS2D || !window.ROS2D.Viewer) return;

    const viewer = new window.ROS2D.Viewer({
      divID: "nav_div",
      width: calculatedCanvasWidth,
      height: calculatedCanvasHeight,
    });

    viewerRef.current = viewer;
    nav2d.canvas = viewer;
  }, []);

  /**
   * Subscribe to "/map" updates only after the topic exists.
   */
  useEffect(() => {
    const topic = mapUpdateTopicRef.current;
    if (!topic) return;

    const handler = () => {
      updateMapPoints((prev) => prev + 1);
    };

    topic.subscribe(handler);

    return () => {
      topic.unsubscribe(handler);
    };
  }, [mapUpdateTopicRef.current]);

  /**
   * On mount: create canvas and request data.
   */
  useEffect(() => {
    createCanvasContainer();

    const mapTopic = mapTopicRef.current;
    if (mapTopic && typeof mapTopic.publish === "function") {
      mapTopic.publish();
    }
  }, [createCanvasContainer]);

  /**
   * Initialize NAV2D map logic.
   * Retry until canvas.scene exists, then init once.
   */
  useEffect(() => {
    if (typeof window === "undefined") return;
    if (!ros) return;

    const nav2d = ensureNav2D();
    if (!nav2d) return;

    createCanvasContainer();

    if (initRetryIntervalRef.current) {
      clearInterval(initRetryIntervalRef.current);
      initRetryIntervalRef.current = null;
    }

    const tryInit = () => {
      const scene = getScene();
      if (typeof nav2d.InitMap !== "function") return;
      if (!scene) return;

      nav2d.InitMap(ros);

      if (initRetryIntervalRef.current) {
        clearInterval(initRetryIntervalRef.current);
        initRetryIntervalRef.current = null;
      }
    };

    tryInit();
    initRetryIntervalRef.current = setInterval(tryInit, 200);

    return () => {
      if (initRetryIntervalRef.current) {
        clearInterval(initRetryIntervalRef.current);
        initRetryIntervalRef.current = null;
      }
    };
  }, [ros, createCanvasContainer]);

  const zoomMap = (value) => {
    if (typeof window === "undefined") return;
    if (!window.ROS2D || !window.ROS2D.ZoomView) return;

    const scene = getScene();
    if (!scene) return;

    const zoomView = new window.ROS2D.ZoomView({
      ros,
      rootObject: scene,
    });

    zoomView.startZoom(300, 200);
    zoomView.zoom(value);
  };

  const shiftMap = (x, y) => {
    const canvas = getCanvas();
    if (!canvas || typeof canvas.shift !== "function") return;
    canvas.shift(x, y);
  };

  return (
    <div
      ref={mapContainer}
      className="flex h-full w-full items-start justify-center"
    >
      <div
        className="relative"
        style={{
          width: canvasWidth ? `${canvasWidth}px` : undefined,
          height: canvasHeight ? `${canvasHeight}px` : undefined,
        }}
      >
        <div className="flex h-full w-full items-center justify-center">
          <div
            id="nav_div"
            ref={mapItem}
            className="mapContainer h-full w-full text-center text-[0px]"
          />
        </div>

        <div className="absolute bottom-8 left-8 grid grid-cols-3 grid-rows-2 justify-items-center gap-1">
          <div />
          <MapButton
            type={"arrow"}
            direction={"top"}
            onBtnClick={() => shiftMap(0, 0.5)}
          />
          <div />

          <MapButton
            type={"arrow"}
            direction={"left"}
            onBtnClick={() => shiftMap(-0.5, 0)}
          />

          <MapButton
            type={"arrow"}
            direction={"bottom"}
            onBtnClick={() => shiftMap(0, -0.5)}
          />

          <MapButton
            type={"arrow"}
            direction={"right"}
            onBtnClick={() => shiftMap(0.5, 0)}
          />

          <MapButton
            type={"zoom"}
            direction={"plus"}
            onBtnClick={() => zoomMap(1)}
          />
          <MapButton
            type={"zoom"}
            direction={"minus"}
            onBtnClick={() => zoomMap(-1)}
          />
        </div>
      </div>
    </div>
  );
});

export default Map;
