import { useEffect, useRef, useState } from "react";

const KEY_DIRECTIONS = {
  ArrowUp: "FORWARD", ArrowDown: "BACKWARD",
  ArrowLeft: "LEFT", ArrowRight: "RIGHT",
};
const KEY_POSITIONS = {
  FORWARD: { x: 0, y: -1 }, BACKWARD: { x: 0, y: 1 },
  LEFT: { x: -1, y: 0 }, RIGHT: { x: 1, y: 0 },
};

export default function ManualJoystick({ disabled, onDirectionChange }) {
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);
  const pointerRef = useRef(null);
  const directionRef = useRef("STOP");
  const callbackRef = useRef(onDirectionChange);
  callbackRef.current = onDirectionChange;

  function changeDirection(direction) {
    if (direction === directionRef.current) return;
    directionRef.current = direction;
    callbackRef.current(direction);
  }

  function reset() {
    pointerRef.current = null;
    setDragging(false);
    setPosition({ x: 0, y: 0 });
    changeDirection("STOP");
  }

  useEffect(() => {
    const hide = () => { if (document.hidden) reset(); };
    window.addEventListener("blur", reset);
    document.addEventListener("visibilitychange", hide);
    return () => {
      window.removeEventListener("blur", reset);
      document.removeEventListener("visibilitychange", hide);
      if (directionRef.current !== "STOP") callbackRef.current("STOP");
    };
  }, []);

  useEffect(() => {
    if (disabled) reset();
  }, [disabled]);

  function move(event) {
    if (disabled || event.pointerId !== pointerRef.current) return;
    const rect = event.currentTarget.getBoundingClientRect();
    const radius = rect.width * 0.25;
    const x = (event.clientX - rect.left - rect.width / 2) / radius;
    const y = (event.clientY - rect.top - rect.height / 2) / radius;
    const distance = Math.hypot(x, y);
    const scale = Math.max(1, distance);
    setPosition({ x: x / scale, y: y / scale });
    // The manual API accepts four directions. The center is a stop zone.
    changeDirection(distance < 0.22 ? "STOP"
      : Math.abs(x) > Math.abs(y) ? (x < 0 ? "LEFT" : "RIGHT")
      : y < 0 ? "FORWARD" : "BACKWARD");
  }

  function release(event) {
    if (event.pointerId !== pointerRef.current) return;
    reset();
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
  }

  return (
    <div className="joystick-control">
      <div
        className={`joystick-base ${dragging ? "dragging" : ""} ${directionRef.current !== "STOP" ? "moving" : ""}`}
        role="group"
        aria-label="AMR L stick. Drag or hold an arrow key to drive. Release to stop."
        aria-disabled={disabled}
        tabIndex={disabled ? -1 : 0}
        onPointerDown={(event) => {
          if (disabled || pointerRef.current !== null || event.button !== 0) return;
          event.preventDefault();
          event.currentTarget.focus();
          pointerRef.current = event.pointerId;
          event.currentTarget.setPointerCapture(event.pointerId);
          setDragging(true);
          move(event);
        }}
        onPointerMove={move}
        onPointerUp={release}
        onPointerCancel={release}
        onLostPointerCapture={release}
        onBlur={reset}
        onKeyDown={(event) => {
          if (disabled || pointerRef.current !== null) return;
          const direction = KEY_DIRECTIONS[event.key];
          if (direction) {
            event.preventDefault();
            setPosition(KEY_POSITIONS[direction]);
            changeDirection(direction);
          } else if (event.key === "Escape" || event.key === " ") {
            event.preventDefault();
            reset();
          }
        }}
        onKeyUp={(event) => {
          if (KEY_DIRECTIONS[event.key] === directionRef.current && pointerRef.current === null) reset();
        }}
      >
        <span className="joystick-mark up" aria-hidden="true">↑</span>
        <span className="joystick-mark down" aria-hidden="true">↓</span>
        <span className="joystick-mark left" aria-hidden="true">←</span>
        <span className="joystick-mark right" aria-hidden="true">→</span>
        <span className="joystick-knob" aria-hidden="true"
          style={{ transform: `translate(${position.x * 68}%, ${position.y * 68}%)` }}>
          L
        </span>
      </div>
      <button type="button" className="button button-ghost joystick-stop" onClick={reset}>BRAKE</button>
    </div>
  );
}
