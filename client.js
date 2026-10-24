window.__ModuleLoader__.load({
  id: '@local/robocasa-control',
  factory(require) {
    const React = require('react')
    const h = React.createElement

    const MONO = 'ui-monospace, SFMono-Regular, Menlo, monospace'
    const BORDER = '1px solid rgba(128, 128, 128, 0.35)'

    /** Sidebar entry icon; the shell owns the button, this owns the artwork. */
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

    function StatusRow(props) {
      return h(
        'div',
        { style: { display: 'flex', gap: 12, flexWrap: 'wrap', fontSize: 12, opacity: 0.85 } },
        props.children,
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

    /** Full monitor panel: live MJPEG, live state, preview mode buttons. */
    function RoboCasaPanel() {
      const [state, setState] = React.useState(null)
      const [error, setError] = React.useState(null)

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

      const setPreview = (mode) => {
        fetch('/robocasa/preview', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mode }),
        }).catch(() => {})
      }

      const open = Boolean(state && state.open)
      const fmt = (value) => (typeof value === 'number' ? value.toFixed(3) : '-')

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

        h('img', {
          src: '/robocasa/frame.mjpg',
          alt: 'RoboCasa live view',
          width: 320,
          height: 240,
          style: {
            width: 320,
            height: 240,
            objectFit: 'cover',
            border: BORDER,
            borderRadius: 8,
            background: '#000',
          },
        }),

        open
          ? h(
              React.Fragment,
              null,
              h(
                StatusRow,
                null,
                h(Pill, null, `step ${state.steps}`),
                h(Pill, null, `success ${String(state.success)}`),
                h(
                  Pill,
                  null,
                  `eef ${fmt(state.eef_pos && state.eef_pos[0])}, ${fmt(state.eef_pos && state.eef_pos[1])}, ${fmt(state.eef_pos && state.eef_pos[2])}`,
                ),
              ),
            )
          : h('div', { style: { fontSize: 12, opacity: 0.7, fontFamily: MONO } }, state ? '打开一个场景后这里会显示状态' : '读取状态中…'),

        error ? h('div', { style: { fontSize: 12, color: '#d9534f' } }, error) : null,

        h(
          'div',
          { style: { display: 'flex', gap: 8, flexWrap: 'wrap', marginTop: 4 } },
          h(Button, { active: open && state.preview === 'shoulder', onClick: () => setPreview('shoulder'), disabled: !open }, '肩载'),
          h(Button, { active: open && state.preview === 'third_person', onClick: () => setPreview('third_person'), disabled: !open }, '第三人称'),
          h(Button, { active: open && state.preview === 'off', onClick: () => setPreview('off'), disabled: !open }, '关闭'),
        ),
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
      },
    }
  },
})
