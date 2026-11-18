const prefix = 'carrier-console:' + self.registration.scope + ':'
const release = prefix + '__RELEASE__'
const files = new Set(__PRECACHE__.map(path => new URL(path, self.registration.scope).href))

self.addEventListener('install', event => {
  event.waitUntil(caches.open(release).then(cache => cache.addAll(
    Array.from(files, url => new Request(url, { cache: 'reload' })),
  )))
})

self.addEventListener('activate', event => {
  event.waitUntil((async () => {
    const keys = await caches.keys()
    await Promise.all(keys.filter(key => key.startsWith(prefix) && key !== release)
      .map(key => caches.delete(key)))
    await self.clients.claim()
  })())
})

self.addEventListener('message', event => {
  if (event.data === 'activate-update') event.waitUntil(self.skipWaiting())
})

self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET') return
  const url = new URL(event.request.url)
  if (event.request.mode === 'navigate') url.search = ''
  if (!files.has(url.href)) return
  event.respondWith(caches.open(release).then(async cache =>
    await cache.match(url.href) ?? fetch(event.request)))
})
