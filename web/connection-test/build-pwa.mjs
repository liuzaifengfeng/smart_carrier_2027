import { createHash } from 'node:crypto'
import { readFile, readdir, writeFile } from 'node:fs/promises'
import { join, relative } from 'node:path'
import { fileURLToPath } from 'node:url'

const dist = new URL('./dist/', import.meta.url)
const template = await readFile(new URL('./service-worker.js', import.meta.url), 'utf8')
const files = (await readdir(dist, { recursive: true, withFileTypes: true }))
  .filter(entry => entry.isFile())
  .map(entry => relative(fileURLToPath(dist), join(entry.parentPath, entry.name)).replaceAll('\\', '/'))
  .filter(path => path !== 'sw.js' && path !== 'index.html' && !path.endsWith('.mjs'))
  .sort()

const hash = createHash('sha256').update(template)
for (const path of files) hash.update(path).update(await readFile(new URL(path, dist)))
await writeFile(new URL('sw.js', dist), template
  .replace('__RELEASE__', hash.digest('hex').slice(0, 16))
  .replace('__PRECACHE__', JSON.stringify(files, null, 2)))
