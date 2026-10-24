/**
 * RoboCasa simulation control for DeepSeek Harness.
 *
 * The plugin owns one long-lived Python worker (worker.py) that holds the
 * MuJoCo/RoboCasa session in memory. Every tool call is a JSON-RPC frame over
 * the worker's stdin/stdout, so physics state survives between tool calls.
 */

import { spawn } from 'node:child_process'
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs'
import { access, constants } from 'node:fs/promises'
import { basename, dirname, join, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

export const name = 'robocasa-control'
export const inject = ['tools', 'webServer']

const HERE = dirname(fileURLToPath(import.meta.url))
const DEFAULT_PYTHON = '/home/zhu/miniconda3/envs/robocasa/bin/python'

const DEFAULT_TASK = 'PickPlaceCounterToCabinet'

const PLUGIN_VERSION = (() => {
  try {
    return JSON.parse(readFileSync(join(HERE, 'package.json'), 'utf8')).version
  } catch {
    return 'unknown'
  }
})()


/** First existing executable among the configured path and known fallbacks. */
async function resolvePython(config) {
  const configured = config.pythonPath || DEFAULT_PYTHON
  if (await isExecutable(configured)) return configured
  for (const candidate of [DEFAULT_PYTHON, '/usr/bin/python3', 'python3']) {
    if (await isExecutable(candidate)) return candidate
  }
  return 'python3'
}

async function isExecutable(path) {
  if (path === 'python3') return true
  try {
    await access(path, constants.X_OK)
    return true
  } catch {
    return false
  }
}

/** Run doctor.py and parse its JSON report. */
function runDoctor(pythonPath, deep, signal) {
  const script = join(HERE, 'doctor.py')
  return new Promise((resolve, reject) => {
    const child = spawn(pythonPath, [script, ...(deep ? ['--deep'] : [])], {
      cwd: HERE,
      env: { ...process.env, PYTHONUNBUFFERED: '1' },
      stdio: ['ignore', 'pipe', 'pipe'],
    })
    let stdout = ''
    let stderr = ''
    const timer = setTimeout(() => {
      child.kill('SIGKILL')
      reject(new Error('environment check timed out after 180 s'))
    }, 180_000)
    const onAbort = () => {
      child.kill('SIGKILL')
      clearTimeout(timer)
      reject(new Error('aborted'))
    }
    signal?.addEventListener('abort', onAbort, { once: true })
    child.stdout.on('data', (chunk) => { stdout += chunk })
    child.stderr.on('data', (chunk) => { stderr += chunk })
    child.on('error', (error) => {
      clearTimeout(timer)
      signal?.removeEventListener('abort', onAbort)
      reject(error)
    })
    child.on('exit', (code) => {
      clearTimeout(timer)
      signal?.removeEventListener('abort', onAbort)
      for (const line of stdout.trim().split('\n').reverse()) {
        if (!line.startsWith('{')) continue
        try {
          resolve(JSON.parse(line))
          return
        } catch {}
      }
      reject(new Error(`environment check produced no report (exit ${code}): ${(stderr || stdout).slice(-600)}`))
    })
  })
}

class SimWorker {
  constructor(config) {
    this.pythonPath = config.pythonPath || DEFAULT_PYTHON
    this.workerScript = config.workerScript || join(HERE, 'worker.py')
    this.cwd = config.cwd || HERE
    this.child = null
    this.seq = 0
    this.pending = new Map()
    this.buffer = ''
    this.stderr = []
    /** Last abnormal worker exit, kept until a later call succeeds. */
    this.crashed = null
  }

  /** Error text that carries the worker's own diagnostics. */
  #decorate(error) {
    const parts = [error instanceof Error ? error.message : String(error)]
    if (this.crashed) {
      parts.unshift(
        `The simulation worker process died (${this.crashed}) and was restarted, `
        + 'so any previously open scene is gone. Call robocasa_open again.',
      )
    }
    const tail = this.tail()
    if (tail) parts.push(`worker stderr (last lines):\n${tail}`)
    return new Error(parts.join('\n\n'))
  }

  #start() {
    if (this.child) return
    const child = spawn(this.pythonPath, [this.workerScript], {
      cwd: this.cwd,
      env: { ...process.env, PYTHONUNBUFFERED: '1' },
      stdio: ['pipe', 'pipe', 'pipe'],
    })
    this.child = child
    child.stdout.setEncoding('utf8')
    child.stderr.setEncoding('utf8')
    child.stdout.on('data', (chunk) => this.#onData(chunk))
    child.stderr.on('data', (chunk) => {
      for (const raw of String(chunk).split('\n')) {
        const line = raw.replace(/\s+$/, '')
        if (line.trim()) this.stderr.push(line)
      }
      if (this.stderr.length > 200) this.stderr.splice(0, this.stderr.length - 200)
    })
    child.on('error', (error) => this.#failAll(error))
    child.on('exit', (code, signal) => {
      // Ignore a worker that a newer one already replaced (close/restart races).
      if (this.child !== child) return
      this.child = null
      this.crashed = signal ? `signal ${signal}` : `exit code ${code}`
      this.#failAll(new Error(`simulation worker exited (code ${code})`))
    })
  }

  #onData(chunk) {
    this.buffer += chunk
    let index
    while ((index = this.buffer.indexOf('\n')) >= 0) {
      const line = this.buffer.slice(0, index).trim()
      this.buffer = this.buffer.slice(index + 1)
      if (!line) continue
      if (!line.startsWith('{')) {
        this.stderr.push(line)
        continue
      }
      let message
      try {
        message = JSON.parse(line)
      } catch {
        continue
      }
      const entry = this.pending.get(message.id)
      if (!entry) continue
      this.pending.delete(message.id)
      clearTimeout(entry.timer)
      if (message.ok) entry.resolve(message.result)
      else entry.reject(new Error(message.error || 'simulation worker error'))
    }
  }

  #failAll(error) {
    for (const [, entry] of this.pending) {
      clearTimeout(entry.timer)
      entry.reject(error)
    }
    this.pending.clear()
  }

  call(method, params = {}, options = {}) {
    this.#start()
    const id = ++this.seq
    const timeoutMs = options.timeoutMs ?? 120_000
    const signal = options.signal
    return new Promise((resolve, reject) => {
      if (signal?.aborted) {
        reject(new Error('aborted'))
        return
      }
      const timer = setTimeout(() => {
        this.pending.delete(id)
        reject(this.#decorate(new Error(`simulation call '${method}' timed out after ${timeoutMs} ms`)))
      }, timeoutMs)
      const onAbort = () => {
        this.pending.delete(id)
        clearTimeout(timer)
        this.stop()
        reject(new Error('aborted'))
      }
      signal?.addEventListener('abort', onAbort, { once: true })
      this.pending.set(id, {
        resolve: (value) => {
          signal?.removeEventListener('abort', onAbort)
          this.crashed = null
          resolve(value)
        },
        reject: (error) => {
          signal?.removeEventListener('abort', onAbort)
          reject(this.#decorate(error))
        },
        timer,
      })
      try {
        this.child.stdin.write(JSON.stringify({ id, method, params }) + '\n')
      } catch (error) {
        this.pending.delete(id)
        clearTimeout(timer)
        reject(this.#decorate(error))
      }
    })
  }

  stop() {
    const child = this.child
    this.child = null
    if (child) {
      try {
        child.stdin.write(JSON.stringify({ id: 0, method: 'close', params: {} }) + '\n')
      } catch {}
      try {
        child.kill('SIGKILL')
      } catch {}
    }
    this.#failAll(new Error('simulation worker stopped'))
  }

  tail() {
    return this.stderr.slice(-12).join('\n')
  }
}

function text(value) {
  return { type: 'text', text: value }
}

function round(value) {
  return typeof value === 'number' ? Math.round(value * 10000) / 10000 : value
}

function yawOf(quat) {
  if (!Array.isArray(quat) || quat.length !== 4) return undefined
  const [x, y, z, w] = quat
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  return Math.round((yaw * 180 / Math.PI) * 10) / 10
}

function poseLine(label, pos) {
  if (!Array.isArray(pos)) return `${label}: unknown`
  return `${label}: x=${round(pos[0])} y=${round(pos[1])} z=${round(pos[2])}`
}

function dimensionLines(dims) {
  if (!dims || typeof dims !== 'object') return []
  const entries = Object.entries(dims)
  if (entries.length === 0) return []
  return ['Object sizes (m):', ...entries.map(([key, size]) =>
    `  ${key}: ${size.map((v) => round(v)).join(' x ')}`)]
}

function objectLines(objects) {
  if (!objects || typeof objects !== 'object') return []
  const entries = Object.entries(objects)
  if (entries.length === 0) return ['Objects: (none reported)']
  return ['Objects (world frame):', ...entries.map(([key, pos]) =>
    `  ${key}: x=${round(pos[0])} y=${round(pos[1])} z=${round(pos[2])}`)]
}

/** Tools advertised to the model. Each entry maps 1:1 to a worker method. */
export function apply(ctx, config = {}) {
  const worker = new SimWorker(config)
  const storageRoot = config.storageRoot || join(process.cwd(), 'robocasa-runs')
  let run = null

  const nowIso = () => new Date().toISOString()

  const saveManifest = () => {
    if (!run) return
    run.manifest.updated_at = nowIso()
    try {
      writeFileSync(join(run.dir, 'manifest.json'), JSON.stringify(run.manifest, null, 2))
    } catch {}
  }

  const bump = (key) => {
    if (!run) return
    run.manifest.stats[key] = (run.manifest.stats[key] || 0) + 1
  }

  const record = (type, extra = {}) => {
    if (!run) return
    run.manifest.events.push({ at: nowIso(), type, ...extra })
    if (run.manifest.events.length > 200) {
      run.manifest.events.splice(0, run.manifest.events.length - 200)
    }
    saveManifest()
  }

  const startRun = (args) => {
    const pad = (n) => String(n).padStart(2, '0')
    const d = new Date()
    const stamp = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}`
      + `-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`
    const base = `${args.task || DEFAULT_TASK}-seed${args.seed ?? 0}-${stamp}`
    let dir
    if (args.dir) {
      dir = resolve(args.dir)
    } else {
      dir = join(storageRoot, base)
      let suffix = 1
      while (existsSync(dir)) dir = join(storageRoot, `${base}-${suffix++}`)
    }
    mkdirSync(join(dir, 'frames'), { recursive: true })
    run = {
      dir,
      manifest: {
        schema: 1,
        run_dir: dir,
        created_at: nowIso(),
        updated_at: nowIso(),
        plugin: { name: '@local/robocasa-control', version: PLUGIN_VERSION },
        environment: {},
        scene: {
          task: args.task || DEFAULT_TASK,
          seed: args.seed ?? 0,
          layout: args.layout ?? null,
          style: args.style ?? null,
          camera: args.camera || 'robot0_agentview_left',
          width: args.width ?? 320,
          height: args.height ?? 320,
        },
        stats: { steps: 0, screenshots: 0, moves: 0, acts: 0, rotates: 0, restarts: 0, resets: 0 },
        events: [],
      },
    }
    saveManifest()
    return dir
  }
  const disposers = []

  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms))

  const sendJson = (res, value, status = 200) => {
    const body = JSON.stringify(value)
    res.writeHead(status, {
      'Content-Type': 'application/json; charset=utf-8',
      'Cache-Control': 'no-store',
      'Content-Length': Buffer.byteLength(body),
    })
    res.end(body)
  }

  /** MJPEG stream: one frame per request cycle, no state kept between clients. */
  const streamFrames = async (req, res) => {
    res.writeHead(200, {
      'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
      'Cache-Control': 'no-store, no-cache, must-revalidate',
      Connection: 'close',
    })
    let closed = false
    req.on('close', () => { closed = true })
    while (!closed) {
      let frame
      try {
        frame = await worker.call('frame', { width: 320, height: 240 }, { timeoutMs: 5000 })
      } catch {
        await sleep(500)
        continue
      }
      const bytes = Buffer.from(frame.image_png_base64, 'base64')
      res.write('--frame\r\nContent-Type: image/png\r\n')
      res.write(`Content-Length: ${bytes.length}\r\n\r\n`)
      res.write(bytes)
      res.write('\r\n')
      await sleep(120)
    }
    res.end()
  }

  const web = ctx.get('webServer')
  if (web) {
    disposers.push(web.register({
      kind: 'exact',
      path: '/robocasa/state',
      handler: async (req, res) => {
        try {
          const value = await worker.call('state', {}, { timeoutMs: 10_000 })
          sendJson(res, value)
        } catch (error) {
          sendJson(res, { open: false, error: String(error?.message ?? error) }, 200)
        }
      },
    }))
    disposers.push(web.register({
      kind: 'exact',
      path: '/robocasa/frame.mjpg',
      handler: streamFrames,
    }))
    disposers.push(web.register({
      kind: 'exact',
      path: '/robocasa/preview',
      handler: async (req, res) => {
        let body = ''
        try {
          for await (const chunk of req) body += chunk
          const payload = body ? JSON.parse(body) : {}
          const value = await worker.call('set_preview', payload, { timeoutMs: 30_000 })
          sendJson(res, value)
        } catch (error) {
          sendJson(res, { error: String(error?.message ?? error) }, 400)
        }
      },
    }))
  }

  const saveImage = async (base64, path) => {
    const attachments = ctx.get('attachments')
    if (!attachments) return undefined
    try {
      return await attachments.saveImage({
        data: new Uint8Array(Buffer.from(base64, 'base64')),
        mediaType: 'image/png',
        name: basename(path),
      })
    } catch {
      return undefined
    }
  }

  const shutdown = () => worker.stop()
  ctx.effect(() => shutdown)

  disposers.push(ctx.tools.register({
    name: 'robocasa_doctor',
    description:
      'Check that the RoboCasa environment is healthy before relying on it. Verifies the Python version, imports and '
      + 'pinned versions of numpy/mujoco/robosuite/robocasa, the asset directory, free disk space and the headless '
      + 'rendering backend, and prints a fix hint for anything that is off. Set deep to also build a tiny scene and '
      + 'render one frame, which proves the whole chain works but adds several seconds.',
    parameters: {
      type: 'object',
      properties: {
        deep: {
          type: 'boolean',
          description: 'Also build a tiny scene and render one frame (adds about 6 seconds).',
          default: false,
        },
      },
      required: [],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => {
        const lines = [
          `Environment: ${value.summary.pass} pass, ${value.summary.warn} warn, ${value.summary.fail} fail — `
            + `${value.ok ? 'healthy' : 'NOT usable yet'}`,
          `Interpreter: ${value.python}`,
        ]
        for (const check of value.checks) {
          const mark = check.status === 'pass' ? '✅' : check.status === 'warn' ? '⚠️' : '❌'
          lines.push(`${mark} ${check.id}: ${check.detail}`)
          if (check.hint) lines.push(`    fix: ${check.hint}`)
        }
        return [text(lines.join('\n'))]
      },
    },
    timeoutMs: 200_000,
    isConcurrencySafe: () => true,
    async execute(args, exec) {
      const python = await resolvePython(config)
      const report = await runDoctor(python, args.deep === true, exec.signal)
      return { ...report, python }
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_open',
    description:
      'Open a RoboCasa kitchen simulation and keep it running in a background process. '
      + 'Call this once before any other robocasa_* tool. Opening a new scene replaces the current one. '
      + 'Returns the end-effector pose and the world positions of the task objects, which you need as '
      + 'targets for robocasa_move. Positions are world coordinates and differ per seed; never assume them.',
    parameters: {
      type: 'object',
      properties: {
        task: {
          type: 'string',
          description: 'RoboCasa task name, for example PickPlaceCounterToCabinet, PickPlaceCounterToSink, or any kitchen task.',
          default: DEFAULT_TASK,
        },
        seed: { type: 'number', description: 'Scene seed; the same seed reproduces the same kitchen layout.', default: 0 },
        layout: { type: 'number', description: 'Optional kitchen layout id (0-9). Omit to sample from the seed.' },
        style: { type: 'number', description: 'Optional kitchen style id (0-9). Omit to sample from the seed.' },
        camera: { type: 'string', description: 'Camera used by robocasa_look.', default: 'robot0_agentview_left' },
        width: { type: 'number', description: 'Render width in pixels (default 320).', default: 320 },
        height: { type: 'number', description: 'Render height in pixels (default 320).', default: 320 },
        preview: {
          type: 'string',
          enum: ['off', 'shoulder', 'third_person'],
          description: 'Preview frame mode for the external viewer window: off, the robot shoulder camera, or a third-person view of the scene.',
          default: 'off',
        },
        dir: {
          type: 'string',
          description: 'Directory for this run (manifest and frames). Defaults to '
            + '<storageRoot>/<task>-seed<N>-<timestamp>; a user-chosen location should be passed here.',
        },
      },
      required: [],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        `Opened ${value.task} (seed ${value.seed}, camera ${value.camera}) in ${value.open_seconds}s.`,
        `Run directory: ${value.run_dir}`,
        poseLine('End effector', value.eef_pos),
        ...objectLines(value.objects),
        ...dimensionLines(value.dimensions),
        'Use robocasa_look to see the scene, then robocasa_move toward an object.',
      ].join('\n'))],
    },
    timeoutMs: 300_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const dir = startRun(args)
      const result = await worker.call(
        'open',
        { ...args, shots_dir: join(dir, 'frames') },
        { timeoutMs: 300_000, signal: exec.signal },
      )
      const status = await worker.call('status', {}, { timeoutMs: 30_000, signal: exec.signal })
      run.manifest.environment = result.versions || {}
      run.manifest.stats.steps = 0
      record('open', { task: result.task, seed: result.seed, camera: result.camera })
      return { ...result, objects: status.objects, dimensions: status.dimensions, run_dir: dir }
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_locate',
    description:
      'Look up a task object or a kitchen fixture by name and return structured geometry instead of raw pixels: '
      + 'world position, bounding-box size, a suggested pre-grasp and grasp point for objects, or interior region '
      + 'centers for fixtures such as a cabinet or sink. Use it before grasping or placing so the target height and '
      + 'depth come from measurement rather than guessing. The current backend reads simulator ground truth and '
      + 'labels every match with source="ground_truth". Pass backend="vision" to run an open-vocabulary detector '
      + 'on the current camera frame instead; that returns several candidates with 2D boxes, scores and world '
      + 'positions back-projected through the depth map, so you can decide which detection is the right instance.',
    parameters: {
      type: 'object',
      properties: {
        query: {
          type: 'string',
          description: 'Case-insensitive substring to match object names or fixture names, for example "obj", '
            + '"apple", "cabinet" or "sink". Use "all" to list everything. With backend="vision" this is a free-text '
            + 'prompt such as "banana" or "cup".',
        },
        backend: {
          type: 'string',
          enum: ['ground_truth', 'vision'],
          description: 'ground_truth (default) reads simulator state; vision runs the detector on the current frame.',
          default: 'ground_truth',
        },
      },
      required: ['query'],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => {
        if (!value.found) {
          return [text(`No object or fixture matched "${value.query}". Try robocasa_status to see what exists.`)]
        }
        if (value.source === 'vision') {
          const lines = [`${value.count} candidate(s) from ${value.model} for "${value.query}"`]
          for (const c of value.candidates) {
            lines.push('')
            lines.push(`  "${c.label}" score=${c.score} box=[${c.box.join(', ')}]`)
            lines.push(`    world: x=${round(c.world[0])} y=${round(c.world[1])} z=${round(c.world[2])} (depth ${c.depth} m)`)
            if (c.grasp_world) lines.push(`    grasp: x=${round(c.grasp_world[0])} y=${round(c.grasp_world[1])} z=${round(c.grasp_world[2])} (surface minus 2 cm)`)
            if (c.visible_extent) lines.push(`    visible extent: ${round(c.visible_extent[0])} x ${round(c.visible_extent[1])} m (projected face, not object height)`)
            if (c.grasp_yaw_deg !== null && c.grasp_yaw_deg !== undefined) {
              lines.push(`    long-axis yaw: ${c.yaw_deg} deg -> suggested gripper yaw: ${c.grasp_yaw_deg} deg`)
            }
          }
          lines.push('')
          lines.push('Verify the instance before moving: another object of the same class may match.')
          return [text(lines.join('\n'))]
        }
        const lines = [`${value.count} match(es) for "${value.query}" (source: ${value.matches[0]?.source ?? 'unknown'})`]
        for (const match of value.matches) {
          lines.push('')
          lines.push(`${match.kind === 'object' ? 'Object' : 'Fixture'} "${match.label}"`)
          if (match.world) lines.push(`  world: x=${round(match.world[0])} y=${round(match.world[1])} z=${round(match.world[2])}`)
          if (match.size) lines.push(`  size: ${match.size.map((v) => round(v)).join(' x ')} m`)
          if (match.top_z !== undefined) lines.push(`  top_z: ${round(match.top_z)}`)
          if (match.world_extent) lines.push(`  horizontal extent: ${round(match.world_extent[0])} x ${round(match.world_extent[1])} m, height ${round(match.world_extent[2])} m`)
          if (match.yaw_deg !== undefined) lines.push(`  long-axis yaw: ${match.yaw_deg} deg`)
          if (match.grasp_yaw_deg !== undefined) lines.push(`  suggested gripper yaw: ${match.grasp_yaw_deg} deg`)
          if (match.pre_grasp) lines.push(`  pre_grasp: ${match.pre_grasp.map((v) => round(v)).join(', ')}`)
          if (match.grasp) lines.push(`  grasp: ${match.grasp.map((v) => round(v)).join(', ')}`)
          if (match.interior) {
            for (const region of match.interior) {
              lines.push(`  interior ${region.region}: ${region.center.map((v) => round(v)).join(', ')}`)
            }
          }
        }
        return [text(lines.join('\n'))]
      },
    },
    timeoutMs: 180_000,
    isConcurrencySafe: () => true,
    async execute(args, exec) {
      const params = { query: args.query ?? 'all', backend: args.backend ?? 'ground_truth' }
      const timeout = params.backend === 'vision' ? 180_000 : 30_000
      return worker.call('locate', params, { timeoutMs: timeout, signal: exec.signal })
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_look',
    description:
      'Capture the current simulated camera view as an image and return it together with the end-effector pose. '
      + 'Call it after opening a scene and after every few movements to see the result of your actions. '
      + 'The image is also saved to disk; the returned path can be re-read later.',
    parameters: {
      type: 'object',
      properties: {
        camera: { type: 'string', description: 'Named camera for this shot, e.g. robot0_eye_in_hand for the wrist view.' },
        free: {
          type: 'object',
          description: 'Render a free camera instead of a named one. Omit to keep using the named camera.',
          properties: {
            lookat: {
              type: 'array',
              items: { type: 'number' },
              description: 'World point to look at; defaults to the end effector.',
            },
            distance: { type: 'number', description: 'Camera distance in metres (default 2.8).' },
            azimuth: { type: 'number', description: 'Azimuth in degrees around the vertical axis (default 200).' },
            elevation: { type: 'number', description: 'Elevation in degrees; negative looks down (default -32).' },
          },
          additionalProperties: false,
        },
        width: { type: 'number', description: 'Render width in pixels; omit to use the scene default.' },
        height: { type: 'number', description: 'Render height in pixels; omit to use the scene default.' },
      },
      required: [],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => {
        const blocks = [text([
          `Camera ${value.camera} at step ${value.steps}.`,
          poseLine('End effector', value.eef_pos),
          `Task success: ${value.success}`,
          `Image: ${value.path}`,
        ].join('\n'))]
        if (value.image && value.image.attachmentId) blocks.push({ type: 'image', attachment: value.image })
        return blocks
      },
    },
    timeoutMs: 60_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const result = await worker.call('look', args, { timeoutMs: 60_000, signal: exec.signal })
      const image = await saveImage(result.image_png_base64, result.path)
      const { image_png_base64: _drop, ...rest } = result
      bump('screenshots')
      if (run) run.manifest.stats.steps = result.steps
      record('screenshot', { camera: result.camera, path: result.path })
      return { ...rest, image, run_dir: run?.dir }
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_move',
    description:
      'Move the robot end effector to a world-space target (x, y, z) using a closed-loop proportional controller, '
      + 'optionally rotating to an absolute orientation given by yaw_deg or a full quaternion. '
      + 'This is the main manipulation primitive: drive toward an object, close the gripper, then lift by moving up. '
      + 'Returns the residual distance; if it is above the tolerance, call again with a nearer target or more steps.',
    parameters: {
      type: 'object',
      properties: {
        x: { type: 'number', description: 'Target world x coordinate.' },
        y: { type: 'number', description: 'Target world y coordinate.' },
        z: { type: 'number', description: 'Target world z coordinate.' },
        gripper: {
          type: 'string',
          enum: ['open', 'close', 'hold'],
          description: 'Gripper command held during the move. close grasps, open releases.',
        },
        max_steps: { type: 'number', description: 'Maximum simulated steps (default 80, max 400).', default: 80 },
        tol: { type: 'number', description: 'Stop when within this distance in meters (default 0.015).', default: 0.015 },
        yaw_deg: {
          type: 'number',
          description: 'Absolute yaw in degrees about the world z axis, keeping the current pitch and roll. '
            + 'Positive is counter-clockwise seen from above.',
        },
        quat: {
          type: 'array',
          items: { type: 'number' },
          description: 'Absolute end-effector orientation as a [x, y, z, w] quaternion in the world frame; overrides yaw_deg.',
        },
        rot_tol: { type: 'number', description: 'Orientation tolerance in radians (default 0.03).', default: 0.03 },
      },
      required: ['x', 'y', 'z'],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        `Moved ${value.steps_used} steps; residual ${value.residual_m} m.`,
        poseLine('Start', value.start),
        poseLine('Target', value.target),
        poseLine('End', value.end),
        `Orientation residual: ${value.rot_residual_rad} rad`,
        `Task success: ${value.success}`,
      ].join('\n'))],
    },
    timeoutMs: 180_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const value = await worker.call('move', args, { timeoutMs: 180_000, signal: exec.signal })
      bump('moves')
      if (run) run.manifest.stats.steps = value.total_steps
      record('move', { target: value.target, residual_m: value.residual_m })
      return value
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_rotate',
    description:
      'Rotate the end effector by a relative amount in degrees about the world axes, using a closed-loop controller '
      + 'that stops when the requested rotation is reached. Use it to align the gripper before a grasp, for example '
      + 'dyaw 90 to turn the wrist a quarter turn. Rotation is applied in the order roll, pitch, yaw composed into one '
      + 'axis-angle step. Returns the residual rotation in radians.',
    parameters: {
      type: 'object',
      properties: {
        droll: { type: 'number', description: 'Roll about the world x axis, in degrees.', default: 0 },
        dpitch: { type: 'number', description: 'Pitch about the world y axis, in degrees.', default: 0 },
        dyaw: { type: 'number', description: 'Yaw about the world z axis, in degrees. Positive is counter-clockwise seen from above.', default: 0 },
        gripper: { type: 'string', enum: ['open', 'close', 'hold'], description: 'Gripper command held during the rotation.' },
        max_steps: { type: 'number', description: 'Maximum simulated steps (default 60, max 400).', default: 60 },
        tol: { type: 'number', description: 'Stop when the remaining rotation is below this many radians (default 0.03, about 1.7 degrees).', default: 0.03 },
      },
      required: [],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        `Rotated ${value.steps_used} steps; residual ${value.residual_rad} rad.`,
        `Yaw: ${yawOf(value.start_quat) ?? 'n/a'} -> ${yawOf(value.end_quat) ?? 'n/a'} deg`,
      ].join('\n'))],
    },
    timeoutMs: 180_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const value = await worker.call('rotate', args, { timeoutMs: 180_000, signal: exec.signal })
      bump('rotates')
      if (run) run.manifest.stats.steps = value.total_steps
      record('rotate', { residual_rad: value.residual_rad })
      return value
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_act',
    description:
      'Apply a raw end-effector translation delta for N simulation steps. Use it for small corrections or for '
      + 'nudging the base when out of reach. Values are normalized controller inputs: about 1.0 means roughly '
      + '5 cm per step. For rotation, use robocasa_rotate instead.',
    parameters: {
      type: 'object',
      properties: {
        dx: { type: 'number', description: 'Delta x in controller units (-1..1).', default: 0 },
        dy: { type: 'number', description: 'Delta y in controller units (-1..1).', default: 0 },
        dz: { type: 'number', description: 'Delta z in controller units (-1..1).', default: 0 },
        gripper: { type: 'string', enum: ['open', 'close', 'hold'], description: 'Gripper command during the steps.' },
        forward: { type: 'number', description: 'Mobile base forward velocity (-1..1).', default: 0 },
        side: { type: 'number', description: 'Mobile base lateral velocity (-1..1).', default: 0 },
        yaw: { type: 'number', description: 'Mobile base rotation (-1..1).', default: 0 },
        torso: {
          type: 'number',
          description: 'Torso height command (-1..1, positive raises the arm). Use it to reach high cabinets; '
            + 'hold 0 to keep the current height.',
        },
        steps: { type: 'number', description: 'Number of steps (default 1, max 200).', default: 1 },
      },
      required: [],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        `Stepped to total ${value.steps} steps.`,
        poseLine('End effector', value.eef_pos),
        `Yaw: ${yawOf(value.eef_quat) ?? 'n/a'} deg`,
        `Task success: ${value.success}`,
      ].join('\n'))],
    },
    timeoutMs: 120_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const value = await worker.call('act', args, { timeoutMs: 120_000, signal: exec.signal })
      bump('acts')
      if (run) run.manifest.stats.steps = value.steps
      record('act', { steps: value.steps })
      return value
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_grasp',
    description:
      'Grasp an object at a world-space point using a multi-hypothesis search. Vision-based orientation is only '
      + 'accurate to roughly 30 degrees, so this tries the given yaw and then the offsets in turn, verifying each '
      + 'attempt by checking whether the object actually starts to follow the gripper. Returns which yaw worked and '
      + 'the per-attempt residuals. Use robocasa_locate first to get the grasp point.',
    parameters: {
      type: 'object',
      properties: {
        x: { type: 'number', description: 'World x of the grasp point.' },
        y: { type: 'number', description: 'World y of the grasp point.' },
        z: { type: 'number', description: 'World z of the grasp point.' },
        yaw_deg: { type: 'number', description: 'Base yaw to try first, usually the long-axis yaw reported by locate.', default: 0 },
        offsets: {
          type: 'array',
          items: { type: 'number' },
          description: 'Additional yaws, relative to yaw_deg, tried in order (default [0, -30, 30]).',
        },
        verify: {
          type: 'string',
          description: 'Simulator object name used only to verify that the grasp lifted something (default "obj").',
          default: 'obj',
        },
        lift: { type: 'number', description: 'Verification lift height in metres (default 0.10).', default: 0.1 },
      },
      required: ['x', 'y', 'z'],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => {
        const lines = [value.success
          ? `Grasped "${value.object}" at yaw ${value.yaw_deg} deg after ${value.attempts.length} attempt(s).`
          : `No attempt lifted "${value.object}" (${value.attempts.length} tried).`]
        for (const a of value.attempts) {
          lines.push(`  yaw ${a.yaw_deg} (offset ${a.offset}): descend ${a.descend_residual_m} m, lift ${a.lift_residual_m} m, object rose ${a.object_z_delta} m`)
        }
        return [text(lines.join('\n'))]
      },
    },
    timeoutMs: 600_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      return worker.call('grasp', args, { timeoutMs: 600_000, signal: exec.signal })
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_preview',
    description:
      'Control the external preview window: turn it off, follow the robot shoulder camera, or show a third-person '
      + 'view of the whole scene. Framing parameters apply to the third-person mode and take effect immediately, '
      + 'without reopening the scene.',
    parameters: {
      type: 'object',
      properties: {
        mode: {
          type: 'string',
          enum: ['off', 'shoulder', 'third_person'],
          description: 'off stops publishing frames; shoulder follows the robot camera; third_person frames the scene.',
        },
        lookat: {
          type: 'array',
          items: { type: 'number' },
          description: 'Optional world point the third-person camera looks at (defaults to the end effector).',
        },
        distance: { type: 'number', description: 'Third-person camera distance in metres (default 2.8).' },
        azimuth: { type: 'number', description: 'Third-person azimuth in degrees (default 200).' },
        elevation: { type: 'number', description: 'Third-person elevation in degrees, negative looks down (default -32).' },
      },
      required: ['mode'],
      additionalProperties: false,
    },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => {
        const lines = [`Preview mode: ${value.mode}`]
        if (value.mode === 'third_person') {
          lines.push(`framing: distance ${round(value.params.dist)} m, azimuth ${round(value.params.az)} deg, elevation ${round(value.params.el)} deg`)
        }
        return [text(lines.join('\n'))]
      },
    },
    timeoutMs: 30_000,
    isConcurrencySafe: () => true,
    async execute(args, exec) {
      return worker.call('set_preview', args, { timeoutMs: 30_000, signal: exec.signal })
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_status',
    description:
      'Report the live simulation state: step count, end-effector pose, task success and the world positions of '
      + 'every task object. Call it when you need fresh target coordinates or to check whether the task is done.',
    parameters: { type: 'object', properties: {}, required: [], additionalProperties: false },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        `${value.task} (seed ${value.seed}) — step ${value.steps}`,
        poseLine('End effector', value.eef_pos),
        `Yaw: ${yawOf(value.eef_quat) ?? 'n/a'} deg`,
        `Task success: ${value.success}`,
        ...objectLines(value.objects),
        ...dimensionLines(value.dimensions),
      ].join('\n'))],
    },
    timeoutMs: 30_000,
    isConcurrencySafe: () => true,
    async execute(args, exec) {
      const value = await worker.call('status', {}, { timeoutMs: 30_000, signal: exec.signal })
      return { ...value, run_dir: run?.dir }
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_reset',
    description: 'Reset the current RoboCasa task to its initial state without rebuilding the scene.',
    parameters: { type: 'object', properties: {}, required: [], additionalProperties: false },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([poseLine('End effector', value.eef_pos), `Task success: ${value.success}`].join('\n'))],
    },
    timeoutMs: 120_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const value = await worker.call('reset', {}, { timeoutMs: 120_000, signal: exec.signal })
      bump('resets')
      if (run) run.manifest.stats.steps = 0
      record('reset')
      return value
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_restart',
    description:
      'Restore the simulation to the exact state captured at robocasa_open or at the last robocasa_reset, '
      + 'without randomizing a new scene. Use it to repeat a trial from identical initial conditions and compare '
      + 'outcomes. Position and orientation are restored bit-exactly from the simulator state.',
    parameters: { type: 'object', properties: {}, required: [], additionalProperties: false },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: (_args, value) => [text([
        'Restored the episode start state.',
        poseLine('End effector', value.eef_pos),
        `Yaw: ${yawOf(value.eef_quat) ?? 'n/a'} deg`,
        'Steps reset to 0.',
      ].join('\n'))],
    },
    timeoutMs: 60_000,
    isConcurrencySafe: () => false,
    async execute(args, exec) {
      const value = await worker.call('restart', {}, { timeoutMs: 60_000, signal: exec.signal })
      bump('restarts')
      if (run) run.manifest.stats.steps = 0
      record('restart')
      return value
    },
  }))

  disposers.push(ctx.tools.register({
    name: 'robocasa_close',
    description: 'Close the running simulation and free its memory. Call it when finished to avoid leaving a Python process running.',
    parameters: { type: 'object', properties: {}, required: [], additionalProperties: false },
    output: {
      schema: { type: 'object', additionalProperties: true },
      render: () => [text('Simulation closed.')],
    },
    timeoutMs: 30_000,
    isConcurrencySafe: () => false,
    async execute() {
      await worker.call('close', {}, { timeoutMs: 30_000 }).catch(() => undefined)
      record('close')
      worker.stop()
      return { closed: true, run_dir: run?.dir }
    },
  }))

  ctx.effect(() => () => {
    for (const dispose of disposers.splice(0)) dispose()
    worker.stop()
  })
}
