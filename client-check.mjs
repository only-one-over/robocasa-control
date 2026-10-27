/**
 * Load the client module the way the web loader does, then render each
 * registered slot component once.
 *
 * `node --check` only parses; it cannot catch a temporal-dead-zone error such as
 * a locale table calling its own lookup function before that function exists.
 * This harness evaluates the factory and every component, so a broken client
 * module fails here instead of in the browser.
 */

import { pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const here = dirname(fileURLToPath(import.meta.url))
const registered = []

const React = {
  createElement: (type, props, ...children) => ({ type, props, children }),
  useState: (value) => [value === undefined ? null : value, () => {}],
  useEffect: () => {},
  useRef: (value) => ({ current: value === undefined ? null : value }),
  useCallback: (fn) => fn,
  Fragment: Symbol('fragment'),
}

globalThis.window = {
  __ModuleLoader__: {
    load(mod) {
      const ctx = mod.factory((name) => (name === 'react' ? React : {}))
      ctx.apply({
        slots: {
          inject: (_name, fn) => fn(),
          register: (spec, component) => {
            registered.push({ spec, component })
            return () => {}
          },
        },
        effect: () => {},
      })
    },
  },
}

await import(pathToFileURL(join(here, 'client.js')).href)

if (registered.length === 0) {
  console.error('FAIL: the module registered no slot components')
  process.exit(1)
}

let failed = 0
for (const { spec, component } of registered) {
  const label = spec.name || spec.key || '(anonymous)'
  try {
    const tree = component({ size: 20, active: false })
    if (!tree) throw new Error('rendered nothing')
    console.log(`PASS  ${label}`)
  } catch (error) {
    failed += 1
    console.log(`FAIL  ${label}: ${error.message}`)
  }
}

console.log(`\n${registered.length - failed}/${registered.length} components rendered`)
process.exit(failed === 0 ? 0 : 1)
