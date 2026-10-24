window.__ModuleLoader__.load({
  id: '@local/robocasa-control',
  factory(require) {
    const React = require('react')
    const h = React.createElement

    const MONO = 'ui-monospace, SFMono-Regular, Menlo, monospace'
    const BORDER = '1px solid rgba(128, 128, 128, 0.35)'
    const CAMERAS = [
      'robot0_agentview_left',
      'robot0_agentview_right',
      'robot0_agentview_center',
      'robot0_eye_in_hand',
      'robot0_frontview',
      'robot0_robotview',
    ]
    const RESOLUTIONS = [
      [320, 240],
      [480, 360],
      [640, 480],
    ]

    function RoboCasaIcon(props) {
      const size = props && props.size ? props.size : 20
      return h(
        'svg',
        {
          viewBox: '0 0 24 24',
          width: size,
          height: size,
          fill: 'none',
          stroke: 'currentColor',
          strokeWidth: 1.6,
          strokeLinecap: 'round',
          strokeLinejoin: 'round',
          'aria-hidden': true,
          style: { display: 'block', opacity: props && props.active ? 1 : 0.75 },
        },
        h('path', { d: 'M3 20h18' }),
        h('path', { d: 'M6 20v-6a4 4 0 0 1 4-4h3' }),
        h('circle', { cx: '6', cy: '13', r: '1.6' }),
        h('path', { d: 'M13 10l4-3' }),
        h('path', { d: 'M17 7l2-3M19 9l2-2' }),
        h('rect', { x: '3', y: '3', width: '4', height: '4', rx: '1' }),
      )
    }

    function Pill(props) {
      return h(
        'span',
        { style: { fontFamily: MONO, padding: '2px 6px', border: BORDER, borderRadius: 6 } },
        props.children,
      )
    }

    function Button(props) {
      return h(
        'button',
        {
          type: 'button',
          onClick: props.onClick,
          disabled: props.disabled,
          style: {
            font: 'inherit',
            fontSize: 12,
            padding: '4px 10px',
            border: BORDER,
            borderRadius: 6,
            background: props.active ? 'rgba(128, 128, 128, 0.22)' : 'transparent',
            color: 'inherit',
            cursor: props.disabled ? 'default' : 'pointer',
            opacity: props.disabled ? 0.45 : 1,
          },
        },
        props.children,
      )
    }

    function Field(props) {
      return h(
        'label',
        { style: { display: 'flex', alignItems: 'center', gap: 8, fontSize: 12 } },
        h('span', { style: { opacity: 0.75, minWidth: 84 } }, props.label),
        props.children,
      )
    }

    function Select(props) {
      return h(
        'select',
        {
          value: String(props.value),
          onChange: (event) => props.onChange(event.target.value),
          disabled: props.disabled,
          style: {
            font: 'inherit',
            fontSize: 12,
            padding: '2px 6px',
            border: BORDER,
            borderRadius: 6,
            background: 'transparent',
            color: 'inherit',
          },
        },
        props.options.map(([value, label]) => h('option', { key: String(value), value: String(value) }, label)),
      )
    }

    function RoboCasaPanel() {
      const [state, setState] = React.useState(null)
      const [settings, setSettings] = React.useState(null)
      const [error, setError] = React.useState(null)
      const [workers, setWorkers] = React.useState([])

      React.useEffect(() => {
        let alive = true
        const tick = async () => {
          try {
            const response = await fetch('/robocasa/state', { cache: 'no-store' })
            const value = await response.json()
            if (alive) {
              setState(value)
              setError(null)
            }
          } catch (err) {
            if (alive) setError(String(err && err.message ? err.message : err))
          }
        }
        tick()
        const timer = setInterval(tick, 1000)
        return () => {
          alive = false
          clearInterval(timer)
        }
      }, [])

      React.useEffect(() => {
        let alive = true
        const tick = async () => {
          try {
            const response = await fetch('/robocasa/workers', { cache: 'no-store' })
            const value = await response.json()
            if (alive) setWorkers(value.workers || [])
          } catch {
            /* keep the last list */
          }
        }
        tick()
        const timer = setInterval(tick, 2000)
        return () => {
          alive = false
          clearInterval(timer)
        }
      }, [])

      React.useEffect(() => {
        let alive = true
        fetch('/robocasa/settings')
          .then((response) => response.json())
          .then((value) => { if (alive) setSettings(value) })
          .catch(() => {})
        return () => { alive = false }
      }, [])

      const patchSettings = (patch) => {
        setSettings((current) => ({ ...(current || {}), ...patch }))
        fetch('/robocasa/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(patch),
        }).catch(() => {})
      }

      const closeWorker = (pid) => {
        fetch('/robocasa/workers', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ pid }),
        }).catch(() => {})
      }

      const open = Boolean(state && state.open)
      const fmt = (value) => (typeof value === 'number' ? value.toFixed(3) : '-')
      const width = settings ? Number(settings.streamWidth) || 320 : 320
      const height = settings ? Number(settings.streamHeight) || 240 : 240

      const config = settings
        ? h(
            'div',
            { style: { display: 'flex', flexDirection: 'column', gap: 8, marginTop: 6 } },
            h('div', { style: { fontSize: 12, opacity: 0.6, letterSpacing: '0.04em' } }, '设置'),
            h(
              Field,
              { label: '画面分辨率' },
              h(Select, {
                value: `${width}x${height}`,
                options: RESOLUTIONS.map(([w, hh]) => [`${w}x${hh}`, `${w}×${hh}`]),
                onChange: (value) => {
                  const [w, hh] = value.split('x').map(Number)
                  patchSettings({ streamWidth: w, streamHeight: hh })
                },
              }),
            ),
            h(
              Field,
              { label: '默认相机' },
              h(Select, {
                value: settings.camera,
                options: CAMERAS.map((name) => [name, name.replace('robot0_', '')]),
                onChange: (value) => patchSettings({ camera: value }),
              }),
            ),
            h(
              Field,
              { label: '检测阈值' },
              h(Select, {
                value: settings.detectorThreshold,
                options: [0.05, 0.1, 0.15, 0.2, 0.3, 0.5].map((v) => [v, v.toFixed(2)]),
                onChange: (value) => patchSettings({ detectorThreshold: Number(value) }),
              }),
            ),
            h(
              Field,
              { label: '预览刷新' },
              h(Select, {
                value: settings.previewEvery,
                options: [1, 2, 4, 8].map((v) => [v, `每 ${v} 步`]),
                onChange: (value) => patchSettings({ previewEvery: Number(value) }),
              }),
            ),
            h(
              'div',
              { style: { fontSize: 11, opacity: 0.55, fontFamily: MONO } },
              '默认相机在下次打开场景时生效，其余立即生效',
            ),
          )
        : null

      return h(
        'div',
        {
          style: {
            display: 'flex',
            flexDirection: 'column',
            gap: 10,
            padding: 14,
            height: '100%',
            boxSizing: 'border-box',
            overflow: 'auto',
            color: 'inherit',
          },
        },
        h(
          'div',
          { style: { display: 'flex', alignItems: 'baseline', gap: 10, flexWrap: 'wrap' } },
          h('strong', { style: { fontSize: 14 } }, 'RoboCasa 仿真'),
          h(
            'span',
            { style: { fontSize: 12, opacity: 0.7, fontFamily: MONO } },
            open ? `${state.task} · seed ${state.seed}` : '未打开场景',
          ),
        ),

        open
          ? h(
              'div',
              { style: { display: 'flex', gap: 12, flexWrap: 'wrap', fontSize: 12, opacity: 0.85 } },
              h(Pill, null, `step ${state.steps}`),
              h(Pill, null, `success ${String(state.success)}`),
              h(
                Pill,
                null,
                `eef ${fmt(state.eef_pos && state.eef_pos[0])}, ${fmt(state.eef_pos && state.eef_pos[1])}, ${fmt(state.eef_pos && state.eef_pos[2])}`,
              ),
            )
          : h(
              'div',
              { style: { fontSize: 12, opacity: 0.7, fontFamily: MONO } },
              state ? '打开一个场景后这里会显示状态' : '读取状态中…',
            ),

        error ? h('div', { style: { fontSize: 12, color: '#d9534f' } }, error) : null,

        h(
          'div',
          { style: { display: 'flex', flexDirection: 'column', gap: 6, marginTop: 4 } },
          h('div', { style: { fontSize: 12, opacity: 0.6, letterSpacing: '0.04em' } }, '运行中的仿真'),
          workers.length === 0
            ? h('div', { style: { fontSize: 12, opacity: 0.6, fontFamily: MONO } }, '没有运行中的 worker 进程')
            : workers.map((item) =>
                h(
                  'div',
                  {
                    key: item.pid,
                    style: {
                      display: 'flex',
                      alignItems: 'center',
                      gap: 8,
                      fontSize: 12,
                      fontFamily: MONO,
                      padding: '3px 6px',
                      border: BORDER,
                      borderRadius: 6,
                      opacity: item.owned ? 1 : 0.72,
                    },
                  },
                  h('span', null, item.owned ? '●' : '○'),
                  h('span', { style: { minWidth: 70 } }, `PID ${item.pid}`),
                  h('span', { style: { minWidth: 64, opacity: 0.75 } }, `${item.ageSec}s`),
                  h('span', { style: { minWidth: 64, opacity: 0.75 } }, `${item.rssMb} MB`),
                  h('span', { style: { flex: 1, opacity: 0.55 } }, item.owned ? '当前' : '孤儿'),
                  h(Button, { onClick: () => closeWorker(item.pid) }, '关闭'),
                ),
              ),
        ),

        config,
      )
    }


    /** Shared polling for the floating widget (state every second, settings once). */
    function useRoboCasaData() {
      const [state, setState] = React.useState(null)
      const [settings, setSettings] = React.useState(null)

      React.useEffect(() => {
        let alive = true
        const tick = async () => {
          try {
            const response = await fetch('/robocasa/state', { cache: 'no-store' })
            const value = await response.json()
            if (alive) setState(value)
          } catch {
            /* keep last known state */
          }
        }
        tick()
        const timer = setInterval(tick, 1500)
        return () => {
          alive = false
          clearInterval(timer)
        }
      }, [])

      React.useEffect(() => {
        let alive = true
        const tick = async () => {
          try {
            const response = await fetch('/robocasa/settings', { cache: 'no-store' })
            const value = await response.json()
            if (alive) setSettings(value)
          } catch {
            /* keep the last settings */
          }
        }
        tick()
        const timer = setInterval(tick, 2000)
        return () => {
          alive = false
          clearInterval(timer)
        }
      }, [])

      return [state, settings]
    }

    /** Floating monitor that coexists with the conversation. */
    function RoboCasaFloat() {
      const [state, settings] = useRoboCasaData()
      const [expanded, setExpanded] = React.useState(true)
      const [showControls, setShowControls] = React.useState(false)
      const [pos, setPos] = React.useState(null)
      const drag = React.useRef(null)

      const width = settings ? Number(settings.streamWidth) || 240 : 240
      const height = settings ? Number(settings.streamHeight) || 180 : 180
      // Follow the configured resolution, capped so the popup still fits a screen.
      const scale = Math.min(1, 480 / width)
      const showWidth = Math.round(width * scale)
      const showHeight = Math.round(height * scale)

      const patchSettings = (patch) => {
        fetch('/robocasa/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(patch),
        }).then((response) => response.json())
          .then((value) => setSettings(value))
          .catch(() => {})
      }

      const onPointerDown = (event) => {
        const box = event.currentTarget.parentElement.getBoundingClientRect()
        drag.current = { dx: event.clientX - box.left, dy: event.clientY - box.top }
        event.currentTarget.setPointerCapture(event.pointerId)
      }
      const onPointerMove = (event) => {
        if (!drag.current) return
        const left = Math.max(8, Math.min(window.innerWidth - 120, event.clientX - drag.current.dx))
        const top = Math.max(8, Math.min(window.innerHeight - 80, event.clientY - drag.current.dy))
        setPos({ left, top })
      }
      const onPointerUp = () => { drag.current = null }

      const chrome = {
        background: 'rgba(18, 22, 28, 0.92)',
        color: '#e6edf3',
        border: '1px solid rgba(255, 255, 255, 0.14)',
        borderRadius: 10,
        boxShadow: '0 10px 30px rgba(0, 0, 0, 0.35)',
        pointerEvents: 'auto',
        overflow: 'hidden',
      }
      const placement = pos
        ? { position: 'fixed', left: pos.left, top: pos.top }
        : { position: 'fixed', right: 18, bottom: 18 }

      if (!expanded) {
        return h(
          'div',
          { style: { ...placement, ...chrome, width: 44, height: 44, display: 'grid', placeItems: 'center' } },
          h(
            'button',
            {
              type: 'button',
              title: '打开 RoboCasa 预览',
              onClick: () => setExpanded(true),
              style: { background: 'transparent', border: 0, color: 'inherit', cursor: 'pointer', padding: 6 },
            },
            h(RoboCasaIcon, { size: 22, active: true }),
          ),
        )
      }

      const open = Boolean(state && state.open)
      return h(
        'div',
        { style: { ...placement, ...chrome, width: showWidth + 20 } },
        h(
          'div',
          {
            onPointerDown,
            onPointerMove,
            onPointerUp,
            style: {
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              gap: 8,
              padding: '5px 8px',
              cursor: 'grab',
              fontSize: 11,
              fontFamily: MONO,
              background: 'rgba(255, 255, 255, 0.05)',
              touchAction: 'none',
            },
          },
          h('span', null, open ? `${state.task.replace('PickPlace', '')} · s${state.seed}` : '未打开场景'),
          h(
            'button',
            {
              type: 'button',
              title: '收起',
              onClick: () => setExpanded(false),
              style: { background: 'transparent', border: 0, color: 'inherit', cursor: 'pointer', fontSize: 13, lineHeight: 1 },
            },
            '—',
          ),
        ),
        open
          ? h('img', {
              key: 'live',
              src: '/robocasa/frame.mjpg',
              alt: 'RoboCasa preview',
              width: showWidth,
              height: showHeight,
              style: { display: 'block', width: showWidth, height: showHeight, objectFit: 'cover', background: '#000' },
            })
          : h(
              'div',
              {
                style: {
                  display: 'grid',
                  placeItems: 'center',
                  width: showWidth,
                  height: showHeight,
                  background: 'rgba(255, 255, 255, 0.05)',
                  fontSize: 11,
                  fontFamily: MONO,
                  opacity: 0.7,
                },
              },
              '未打开场景',
            ),
        h(
          'div',
          {
            style: {
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              gap: 6,
              padding: '3px 8px',
              fontSize: 11,
              fontFamily: MONO,
              opacity: 0.85,
            },
          },
          h('span', null, open ? `step ${state.steps} · ${state.success ? 'success' : 'running'}` : '未打开场景'),
          settings
            ? h(
                'button',
                {
                  type: 'button',
                  onClick: () => setShowControls((value) => !value),
                  title: '视角设置',
                  style: {
                    background: 'transparent',
                    border: 0,
                    color: 'inherit',
                    cursor: 'pointer',
                    fontSize: 12,
                    padding: 0,
                  },
                },
                showControls ? '×' : '⚙',
              )
            : null,
        ),

        showControls && settings
          ? h(
              'div',
              {
                style: {
                  display: 'grid',
                  gridTemplateColumns: '1fr 1fr',
                  gap: 4,
                  padding: '4px 8px 6px',
                  fontSize: 11,
                  fontFamily: MONO,
                },
              },
              h(Select, {
                value: settings.viewMode,
                options: [['third_person', '第三人称'], ['shoulder', '肩载']],
                onChange: (value) => patchSettings({ viewMode: value }),
              }),
              h(Select, {
                value: settings.azimuth,
                options: [0, 45, 90, 135, 180, 225, 270, 315].map((v) => [v, `方位 ${v}°`]),
                onChange: (value) => patchSettings({ azimuth: Number(value) }),
              }),
              h(Select, {
                value: settings.elevation,
                options: [-15, -25, -32, -45, -62].map((v) => [v, `俯角 ${v}°`]),
                onChange: (value) => patchSettings({ elevation: Number(v) }),
              }),
              h(Select, {
                value: settings.distance,
                options: [1.8, 2.2, 2.8, 3.6, 4.5].map((v) => [v, `距离 ${v}m`]),
                onChange: (value) => patchSettings({ distance: Number(v) }),
              }),
            )
          : null,
      )
    }

    return {
      inject: ['slots'],
      apply(ctx) {
        ctx.slots.inject('sidebar.panellist', () =>
          ctx.slots.register(
            { name: 'sidebar.panellist', id: 'robocasa', order: 20, label: 'RoboCasa' },
            RoboCasaIcon,
          ),
        )
        ctx.slots.inject('main', () =>
          ctx.slots.register({ name: 'main', key: 'robocasa' }, RoboCasaPanel),
        )
        ctx.slots.inject('shell.overlay', () =>
          ctx.slots.register(
            { name: 'shell.overlay', id: 'robocasa-float', order: 10, label: 'RoboCasa 悬浮预览' },
            RoboCasaFloat,
          ),
        )
      },
    }
  },
})
