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

const Map = forwardRef((props, ref) => {
  const ros = useContext(RosContext);
  const mapContainer = useRef();
  const mapItem = useRef();

  const [canvasWidth, setCanvasWidth] = useState();
  const [canvasHeight, setCanvasHeight] = useState();
  // eslint-disable-next-line no-unused-vars
  const [mapPoints, updateMapPoints] = useState(0);

  useImperativeHandle(ref, () => ({
    getMapRef: () => mapItem.current,
  }));

  const mapTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.WP_REQ,
      messageType: "std_msgs/Empty",
    }),
  );

  const mapUpdateTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: "/map",
      messageType: "nav_msgs/OccupancyGrid",
    }),
  );

  const createCanvasContainer = useCallback(() => {
    /**
     * ROS2D.Viewer - function, that selects an element with id='nav_div' and adds the canvas
     * block with map inside. This block can be cleared only with full reload of the page, but useEffect works
     * during every Map.jsx remounting even without page reload.
     * As a result the function will be called again while we still have map inside. Thats why we call function
     * only if component has less than 1 card.
     */
    const mapElements = mapItem.current.childNodes;
    if (mapElements.length < 1) {
      const container = mapContainer.current;
      const containerWidth = container.offsetWidth;
      const containerHeight = container.offsetHeight;

      // Определяем максимальную высоту, которую может иметь канвас
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

      const viewer = new window.ROS2D.Viewer({
        divID: "nav_div",
        width: calculatedCanvasWidth,
        height: calculatedCanvasHeight,
      });
      window.NAV2D.canvas = viewer;
    }
  }, []);

  /* Reload map component after changing map or route  */
  useEffect(() => {
    const mapUpdateCurrentTopic = mapUpdateTopic.current;
    mapUpdateCurrentTopic.subscribe(() => {
      const newValue = mapPoints + 1;
      updateMapPoints(newValue);
    });
    return () => {
      mapUpdateCurrentTopic.unsubscribe();
    };
  }, []);

  useEffect(() => {
    mapTopic.current.publish();
    createCanvasContainer();
  }, [createCanvasContainer]);

  useEffect(() => {
    window.NAV2D.InitMap(ros);
  }, [ros]);

  const zoomMap = (value) => {
    const zoomTopic = new window.ROS2D.ZoomView({
      ros,
      rootObject: window.NAV2D.canvas.scene,
    });
    zoomTopic.startZoom(300, 200);
    zoomTopic.zoom(value);
  };

  const shiftMap = (x, y) => {
    window.NAV2D.canvas.shift(x, y);
  };

  return (
    <div
      ref={mapContainer}
      className="flex h-full w-full items-start justify-center"
    >
      <div
        className="relative"
        style={{
          width: `${canvasWidth}px`,
          height: `${canvasHeight}px`,
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
        </div>
        <div className="absolute bottom-8 right-8 flex flex-col gap-1">
          <MapButton type={"plus"} onBtnClick={() => zoomMap(1.2)} />
          <MapButton type={"minus"} onBtnClick={() => zoomMap(0.8)} />
        </div>
      </div>
    </div>
  );
});

export default Map;
