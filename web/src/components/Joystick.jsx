import React, { useRef, useEffect, useState, useCallback } from "react";
import { Joystick } from "react-joystick-component";

import { useRuntimeConfig } from "../app/App";
import { useMotionGate } from "../shared/motion/motionGate";
import { holdVelocity, releaseVelocity } from "../shared/motion/uiCommand";

// W3C Gamepad API standard mapping: left stick is axes[0] (x, +right) and
// axes[1] (y, +down — inverted vs. the joystick lib's evt.y, which is +up).
const GAMEPAD_DEADZONE = 0.12;
const applyDeadzone = (value) => (Math.abs(value) > GAMEPAD_DEADZONE ? value : 0);

const JoystickComponent = ({ maxSpeed, compact = false }) => {
  const { config } = useRuntimeConfig();
  const gate = useMotionGate();
  const joysticRef = useRef();
  const [size, setSize] = useState(180);
  const [stickSize, setStickSize] = useState(120);
  const [gamepadConnected, setGamepadConnected] = useState(false);

  const effectiveMax = maxSpeed ?? config.maxLinearSpeed;
  const disabled = !gate.canDrive;

  // Both inputs go through shared/motion/uiCommand.js, the only velocity
  // publisher: it stamps each command, publishes on /ui/cmd_vel at 20 Hz while
  // held and sends one zero on release. Nothing is published while the
  // motion gate is closed.
  const driveFrom = useCallback(
    (source, evt) =>
      holdVelocity(source, {
        linear: evt.y * effectiveMax,
        angular: -evt.x * config.maxAngularSpeed,
      }),
    [effectiveMax, config.maxAngularSpeed],
  );

  // Same rule as the gamepad below: a drag that was refused (gate closed)
  // drives again only after the stick is released and pressed again.
  const joystickNeedsReleaseRef = useRef(false);
  const handleJoysticMove = useCallback(
    (evt) => {
      if (joystickNeedsReleaseRef.current) return;
      if (!driveFrom("joystick", evt).ok) joystickNeedsReleaseRef.current = true;
    },
    [driveFrom],
  );
  const handleStop = useCallback(() => {
    joystickNeedsReleaseRef.current = false;
    releaseVelocity("joystick");
  }, []);

  // Gamepad support: an alternate input feeding the same uiCommand path the
  // on-screen joystick uses, with the same topic and speed limits. Polled
  // via rAF (the Gamepad API has no change events for stick movement),
  // edge-triggered so the release fires once per release, not every frame.
  useEffect(() => {
    const gamepadActiveRef = { current: false };
    const needsNeutralRef = { current: false };
    let rafId;

    const poll = () => {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      const gp = pads && pads[0];
      const x = gp ? applyDeadzone(gp.axes[0] || 0) : 0;
      const y = gp ? applyDeadzone(gp.axes[1] || 0) : 0;

      if (x !== 0 || y !== 0) {
        // A stick held while the gate was closed (stop, unknown lock, clock
        // not synced) must return to neutral before it drives again, so
        // Resume never starts motion from a stick that is already deflected.
        if (!needsNeutralRef.current) {
          gamepadActiveRef.current = true;
          if (!driveFrom("gamepad", { x, y: -y }).ok) {
            needsNeutralRef.current = true;
            gamepadActiveRef.current = false;
          }
        }
      } else {
        needsNeutralRef.current = false;
        if (gamepadActiveRef.current) {
          gamepadActiveRef.current = false;
          releaseVelocity("gamepad");
        }
      }
      rafId = requestAnimationFrame(poll);
    };
    rafId = requestAnimationFrame(poll);

    const onConnect = () => setGamepadConnected(true);
    const onDisconnect = () => setGamepadConnected(false);
    window.addEventListener("gamepadconnected", onConnect);
    window.addEventListener("gamepaddisconnected", onDisconnect);
    setGamepadConnected((navigator.getGamepads?.() || []).some(Boolean));

    return () => {
      cancelAnimationFrame(rafId);
      window.removeEventListener("gamepadconnected", onConnect);
      window.removeEventListener("gamepaddisconnected", onDisconnect);
    };
  }, [driveFrom]);

  useEffect(() => {
    if (!joysticRef.current) return;
    const blockWidth = joysticRef.current.getBoundingClientRect().width;
    setSize(blockWidth);
    setStickSize(blockWidth * 0.66);

    return () => {
      releaseVelocity("joystick");
      releaseVelocity("gamepad");
    };
  }, []);

  return (
    <div
      ref={joysticRef}
      className={`relative ${
        compact
          ? "h-[82px] w-[82px] lg:h-[88px] lg:w-[88px] 2xl:h-[96px] 2xl:w-[96px]"
          : "h-[100px] w-[100px] lg:h-[140px] lg:w-[140px] 2xl:h-[180px] 2xl:w-[180px]"
      }`}
    >
      <div className="absolute">
        <Joystick
          throttle={150}
          size={size}
          stickSize={stickSize}
          baseColor="#17171d"
          stickColor="#8b5cf6"
          move={handleJoysticMove}
          stop={handleStop}
          disabled={disabled}
        />
      </div>
      {disabled && (
        <div
          role="status"
          data-testid="manual-drive-disabled"
          className="absolute inset-0 flex items-center justify-center rounded-full bg-black/60 p-1 text-center font-[RobotoMono] text-[9px] leading-tight text-statusYellow"
        >
          {gate.reason}
        </div>
      )}
      {gamepadConnected && (
        <span
          title="Gamepad connected — left stick drives"
          className="absolute -right-1 -top-1 h-2.5 w-2.5 rounded-full bg-statusGreen shadow-[0_0_0_2px_rgba(0,0,0,0.4)]"
        />
      )}
    </div>
  );
};

export default JoystickComponent;
