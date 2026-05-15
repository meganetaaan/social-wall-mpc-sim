import '@testing-library/jest-dom/vitest'

HTMLCanvasElement.prototype.getContext = (() => ({
  arc: () => {},
  beginPath: () => {},
  clearRect: () => {},
  fill: () => {},
  fillRect: () => {},
  fillText: () => {},
  lineTo: () => {},
  moveTo: () => {},
  setTransform: () => {},
  stroke: () => {},
  get canvas() {
    return document.createElement('canvas')
  },
})) as unknown as typeof HTMLCanvasElement.prototype.getContext
