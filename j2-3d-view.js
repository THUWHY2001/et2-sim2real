(() => {
  'use strict';

  const ZOOM_MIN = 0.55;
  const ZOOM_MAX = 2.4;
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const clampZoom = value => clamp(value, ZOOM_MIN, ZOOM_MAX);

  function projectPoint(point, view) {
    const center = view.center || {x: 0, y: 0, z: 0};
    const dx = point.x - center.x;
    const dy = point.y - center.y;
    const dz = point.z - center.z;
    const cy = Math.cos(view.yaw || 0);
    const sy = Math.sin(view.yaw || 0);
    const cp = Math.cos(view.pitch || 0);
    const sp = Math.sin(view.pitch || 0);
    const horizontal = cy * dx - sy * dy;
    const yawDepth = sy * dx + cy * dy;
    const vertical = cp * dz - sp * yawDepth;
    const depth = sp * dz + cp * yawDepth;
    const scale = (view.scale || 1) * (view.zoom || 1);
    const perspective = 900 / Math.max(360, 900 + depth);
    return {
      x: (view.width || 0) / 2 + horizontal * scale * perspective,
      y: (view.height || 0) / 2 - vertical * scale * perspective,
      depth,
      perspective,
    };
  }

  function create(canvas, options = {}) {
    if (!canvas || typeof canvas.getContext !== 'function') throw new TypeError('J2 3D view requires a canvas');
    const context = canvas.getContext('2d');
    const initial = {yaw: -0.62, pitch: -0.18, zoom: 1};
    const state = {...initial};
    let latest = null;
    let dragging = false;
    let pointerId = null;
    let lastX = 0;
    let lastY = 0;
    let resizeObserver = null;

    const handlers = {};
    const on = (name, handler, settings) => {
      handlers[name] = handler;
      canvas.addEventListener(name, handler, settings);
    };

    const resize = () => {
      const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : {width: canvas.clientWidth, height: canvas.clientHeight};
      const width = Math.max(320, Math.round(rect.width || canvas.clientWidth || 800));
      const height = Math.max(360, Math.round(rect.height || canvas.clientHeight || 520));
      const ratio = Math.max(1, Math.min(2, globalThis.devicePixelRatio || 1));
      if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
        canvas.width = Math.round(width * ratio);
        canvas.height = Math.round(height * ratio);
      }
      if (context.setTransform) context.setTransform(ratio, 0, 0, ratio, 0, 0);
      return {width, height};
    };

    const point3 = (point, y = 93) => ({x: point.x, y: Number.isFinite(point.y) ? point.y : y, z: point.z});
    const axis = (point, halfLength) => [point3(point, 93 - halfLength), point3(point, 93 + halfLength)];

    function render(points) {
      latest = points;
      const {width, height} = resize();
      if (!context || !points) return;
      context.clearRect(0, 0, width, height);

      const nodes = {
        j2: point3(points.j2),
        axis1: point3(points.axis1),
        axis2: point3(points.axis2),
        axis3: point3(points.axis3),
        strokeEnd: point3(points.strokeEnd),
        point2: point3(points.point2),
        lower: point3(points.lower),
      };
      const all = Object.values(nodes);
      const bounds = all.reduce((box, point) => ({
        minX: Math.min(box.minX, point.x), maxX: Math.max(box.maxX, point.x),
        minY: Math.min(box.minY, point.y), maxY: Math.max(box.maxY, point.y),
        minZ: Math.min(box.minZ, point.z), maxZ: Math.max(box.maxZ, point.z),
      }), {minX: Infinity, maxX: -Infinity, minY: 52, maxY: 134, minZ: Infinity, maxZ: -Infinity});
      const center = {x: (bounds.minX + bounds.maxX) / 2, y: 93, z: (bounds.minZ + bounds.maxZ) / 2};
      const spanX = Math.max(1, bounds.maxX - bounds.minX + 90);
      const spanZ = Math.max(1, bounds.maxZ - bounds.minZ + 70);
      const scale = Math.min((width - 110) / spanX, (height - 90) / spanZ);
      const view = {...state, width, height, center, scale};
      const projected = Object.fromEntries(Object.entries(nodes).map(([key, value]) => [key, projectPoint(value, view)]));

      context.lineCap = 'round';
      context.lineJoin = 'round';

      const segments = [
        {a: 'axis1', b: 'axis2', color: '#475569', width: 8},
        {a: 'axis2', b: 'axis3', color: '#6b506b', width: 8},
        {a: 'axis3', b: 'lower', color: '#475569', width: 10},
        {a: 'lower', b: 'point2', color: '#475569', width: 7},
        {a: 'j2', b: 'axis3', color: '#368451', width: 7},
        {a: 'axis2', b: 'strokeEnd', color: '#e87924', width: 5},
        {a: 'strokeEnd', b: 'point2', color: '#e9a56b', width: 11},
      ].sort((left, right) => ((projected[left.a].depth + projected[left.b].depth) - (projected[right.a].depth + projected[right.b].depth)));

      for (const segment of segments) {
        const a = projected[segment.a];
        const b = projected[segment.b];
        context.beginPath();
        context.moveTo(a.x, a.y);
        context.lineTo(b.x, b.y);
        context.strokeStyle = segment.color;
        context.lineWidth = segment.width * (a.perspective + b.perspective) / 2;
        context.stroke();
      }

      const axes = [
        {key: 'j2', half: 52.25, color: '#1f5faa'},
        {key: 'axis1', half: 45.25, color: '#1f5faa'},
        {key: 'axis2', half: 20.3, color: '#1f5faa'},
        {key: 'axis3', half: 20, color: '#1f5faa'},
        {key: 'point2', half: 18, color: '#e87924'},
      ].map(item => {
        const ends = axis(nodes[item.key], item.half).map(point => projectPoint(point, view));
        return {...item, ends, depth: (ends[0].depth + ends[1].depth) / 2};
      }).sort((a, b) => a.depth - b.depth);

      for (const item of axes) {
        context.beginPath();
        context.moveTo(item.ends[0].x, item.ends[0].y);
        context.lineTo(item.ends[1].x, item.ends[1].y);
        context.strokeStyle = item.color;
        context.lineWidth = 7;
        context.stroke();
      }

      const labels = {
        j2: 'J2 等效旋转轴', axis1: '轴1', axis2: '轴2 / 上端滑动轴承', axis3: '轴3',
        strokeEnd: '伸缩段末端', point2: '下端滑动轴承', lower: '下端点',
      };
      context.font = '600 13px system-ui, sans-serif';
      context.textBaseline = 'middle';
      for (const [key, label] of Object.entries(labels)) {
        const p = projected[key];
        const radius = key === 'strokeEnd' ? 5 : 7;
        context.beginPath();
        context.arc(p.x, p.y, radius, 0, Math.PI * 2);
        context.fillStyle = '#fff';
        context.fill();
        context.strokeStyle = key === 'point2' || key === 'strokeEnd' ? '#e87924' : '#2f64ad';
        context.lineWidth = 3;
        context.stroke();
        const textWidth = context.measureText ? context.measureText(label).width : label.length * 13;
        const labelX = clamp(p.x + 12, 6, width - textWidth - 8);
        const labelY = clamp(p.y + (key === 'axis3' || key === 'lower' ? 17 : -15), 14, height - 14);
        context.fillStyle = 'rgba(255,255,255,.88)';
        context.fillRect(labelX - 3, labelY - 9, textWidth + 6, 18);
        context.fillStyle = '#172033';
        context.fillText(label, labelX, labelY);
      }

      const origin = {x: 52, y: height - 42};
      const triad = [
        {label: 'X', vector: {x: 34, y: 0, z: 0}, color: '#d04a45'},
        {label: 'Y', vector: {x: 0, y: 34, z: 0}, color: '#3d8b5c'},
        {label: 'Z', vector: {x: 0, y: 0, z: 34}, color: '#356bb3'},
      ];
      const triadCenter = {x: 0, y: 0, z: 0};
      const triadView = {...view, center: triadCenter, width: 0, height: 0, scale: .8, zoom: 1};
      for (const item of triad) {
        const end = projectPoint(item.vector, triadView);
        context.beginPath();
        context.moveTo(origin.x, origin.y);
        context.lineTo(origin.x + end.x, origin.y + end.y);
        context.strokeStyle = item.color;
        context.lineWidth = 2.5;
        context.stroke();
        context.fillStyle = item.color;
        context.fillText(item.label, origin.x + end.x + 4, origin.y + end.y);
      }
    }

    const redraw = () => latest && render(latest);
    on('pointerdown', event => {
      dragging = true;
      pointerId = event.pointerId;
      lastX = event.clientX;
      lastY = event.clientY;
      if (canvas.setPointerCapture) canvas.setPointerCapture(pointerId);
    });
    on('pointermove', event => {
      if (!dragging || event.pointerId !== pointerId) return;
      state.yaw += (event.clientX - lastX) * 0.008;
      state.pitch = clamp(state.pitch + (event.clientY - lastY) * 0.008, -1.15, 1.15);
      lastX = event.clientX;
      lastY = event.clientY;
      redraw();
    });
    const stopDrag = event => {
      if (event.pointerId !== undefined && pointerId !== null && event.pointerId !== pointerId) return;
      dragging = false;
      pointerId = null;
    };
    on('pointerup', stopDrag);
    on('pointercancel', stopDrag);
    on('wheel', event => {
      if (event.preventDefault) event.preventDefault();
      state.zoom = clampZoom(state.zoom * Math.exp(-event.deltaY * 0.001));
      redraw();
    }, {passive: false});
    canvas.style.touchAction = 'none';
    canvas.style.cursor = 'grab';

    if (typeof ResizeObserver === 'function') {
      resizeObserver = new ResizeObserver(redraw);
      resizeObserver.observe(canvas);
    }

    return {
      render,
      sideView() { state.yaw = 0; state.pitch = 0; state.zoom = 1; redraw(); },
      resetView() { Object.assign(state, initial); redraw(); },
      destroy() {
        for (const [name, handler] of Object.entries(handlers)) canvas.removeEventListener(name, handler);
        if (resizeObserver) resizeObserver.disconnect();
        latest = null;
      },
    };
  }

  window.J2ThreeView = {create, projectPoint, clampZoom};
})();
