#!/usr/bin/env python3
"""
uwb_all.py — UWB Real-Time Tracking
  - HTTP : http://0.0.0.0:8080  (local + tailscale)
  - WS   : ws://0.0.0.0:8765
  - TCP  : nhận data từ 2 tag

pip install websockets
python uwb_all.py
"""

import asyncio, json, socket, threading, time
import http.server, webbrowser
from queue import Queue
import websockets
import webbrowser

# ══════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════
TAG_SOURCES = {
    "TAG1": {"host": "100.102.252.35", "port": 9001},
    "TAG2": {"host": "100.102.252.35", "port": 9002},
}
WS_PORT   = 8765
HTTP_PORT = 8080

ANCHORS = [
    {"x": 0,     "y": 0,    "label": "A0"},
    {"x": 0,     "y": 4400, "label": "A1"},
    {"x": 14800, "y": 4400, "label": "A2"},
    {"x": 14800, "y": 0,    "label": "A3"},
]

TAILSCALE_IP   = "100.103.224.66"

# ══════════════════════════════════════════
#  UTILS
# ══════════════════════════════════════════
def get_ips():
    local = "localhost"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local = s.getsockname()[0]
        s.close()
    except Exception:
        pass
    return {"local": local, "tailscale": TAILSCALE_IP or None}

# ══════════════════════════════════════════
#  HTML
# ══════════════════════════════════════════
def build_html(ws_port, anchors):
    anchors_js = json.dumps(anchors)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UWB Tracking</title>
<style>
  *, *::before, *::after {{ box-sizing:border-box; margin:0; padding:0; }}
  :root {{
    --bg:#0d1117; --bg2:#161b22; --bg3:#21262d;
    --border:#30363d; --text:#c9d1d9; --muted:#8b949e; --dim:#484f58;
    --tag1:#ff5050; --tag2:#50d278;
  }}
  html,body {{ width:100%; height:100%; background:var(--bg); color:var(--text);
    font-family:'Consolas','Courier New',monospace; overflow:hidden; }}
  #app {{ display:flex; width:100vw; height:100vh; }}
  #canvas-wrap {{ flex:1; position:relative; overflow:hidden; }}
  canvas {{ display:block; cursor:crosshair; }}
  #sidebar {{ width:220px; flex-shrink:0; background:var(--bg2);
    border-left:1px solid var(--border); display:flex; flex-direction:column;
    padding:16px 12px 12px; gap:12px; overflow-y:auto; }}
  .lbl {{ color:var(--dim); font-size:10px; letter-spacing:2px; text-transform:uppercase; }}
  .sep {{ height:1px; background:var(--border); flex-shrink:0; }}
  .card {{ border:1px solid var(--border); border-radius:6px; background:var(--bg);
    padding:10px; display:flex; flex-direction:column; gap:4px; }}
  .card[data-tag="TAG1"] {{ border-color:#ff505055; }}
  .card[data-tag="TAG2"] {{ border-color:#50d27855; }}
  .card-header {{ display:flex; align-items:center; justify-content:space-between; }}
  .tag-name {{ font-weight:bold; font-size:13px; }}
  .tag-name[data-tag="TAG1"] {{ color:var(--tag1); }}
  .tag-name[data-tag="TAG2"] {{ color:var(--tag2); }}
  .dot {{ font-size:13px; transition:color .3s; color:var(--dim); }}
  .dot.connected    {{ color:#3fb950; }}
  .dot.connecting   {{ color:#f0883e; animation:blink 1s infinite; }}
  .dot.disconnected {{ color:#f85149; }}
  @keyframes blink {{ 0%,100%{{opacity:1}} 50%{{opacity:.3}} }}
  .card-sub {{ color:var(--dim); font-size:10px; }}
  .card-coord {{ color:var(--muted); font-size:11px; line-height:1.7; white-space:pre; }}
  .card-prob {{ font-size:11px; line-height:1.6; }}
  .prob-bar-wrap {{ height:4px; background:var(--bg3); border-radius:2px; margin-top:2px; }}
  .prob-bar {{ height:4px; border-radius:2px; background:#3fb950; transition:width .3s; }}
  btn-row {{ display:flex; gap:6px; }}
  button {{ background:var(--bg3); color:var(--muted); border:1px solid var(--border);
    border-radius:4px; padding:6px 8px; font-family:inherit; font-size:11px;
    cursor:pointer; transition:background .15s,color .15s; width:100%; }}
  button:hover {{ background:var(--border); color:var(--text); }}
  #stats {{ color:var(--dim); font-size:10px; line-height:1.9; margin-top:auto; }}
  #ws-bar {{ position:absolute; bottom:10px; left:10px; background:#00000099;
    border:1px solid var(--border); border-radius:4px; padding:4px 12px;
    font-size:10px; color:var(--dim); pointer-events:none; }}
</style>
</head>
<body>
<div id="app">
  <div id="canvas-wrap">
    <canvas id="map"></canvas>
    <div id="ws-bar">WS: connecting…</div>
  </div>
  <div id="sidebar">
    <div class="lbl">TAGS</div>

    <div class="card" data-tag="TAG1">
      <div class="card-header">
        <span class="tag-name" data-tag="TAG1">TAG 1</span>
        <span class="dot disconnected" id="dot-TAG1">●</span>
      </div>
      <div class="card-sub" id="ip-TAG1"></div>
      <div class="card-coord" id="coord-TAG1">X: —\nY: —</div>
      <div class="card-prob" id="prob-TAG1" style="display:none">
        <span id="prob-label-TAG1">OnHand: —</span>
        <div class="prob-bar-wrap"><div class="prob-bar" id="prob-bar-TAG1" style="width:0%"></div></div>
      </div>
    </div>

    <div class="card" data-tag="TAG2">
      <div class="card-header">
        <span class="tag-name" data-tag="TAG2">TAG 2</span>
        <span class="dot disconnected" id="dot-TAG2">●</span>
      </div>
      <div class="card-sub" id="ip-TAG2"></div>
      <div class="card-coord" id="coord-TAG2">X: —\nY: —</div>
      <div class="card-prob" id="prob-TAG2" style="display:none">
        <span id="prob-label-TAG2">OnHand: —</span>
        <div class="prob-bar-wrap"><div class="prob-bar" id="prob-bar-TAG2" style="width:0%"></div></div>
      </div>
    </div>

    <div class="sep"></div>
    <div class="lbl">CONTROLS</div>
    <button id="btn-clear">Clear Trails</button>
    <button id="btn-trail">Trail: ON</button>
    <button id="btn-grid">Grid: ON</button>

    <div class="sep"></div>
    <div id="stats">TAG1: 0 pts\nTAG2: 0 pts\n\nWaiting…</div>
  </div>
</div>

<script>
const WS_URL    = "ws://" + location.hostname + ":{ws_port}";
const ANCHORS   = {anchors_js};
const TAG_STYLE = {{
  TAG1: {{ color:[255,80,80],  name:"TAG 1" }},
  TAG2: {{ color:[80,210,120], name:"TAG 2" }},
}};
const TAG_SOURCES = {json.dumps({k: v for k, v in TAG_SOURCES.items()})};

const TRAIL_MAX = 800;
const MARGIN    = 55;
const WORLD_X   = [-500, 15300];
const WORLD_Y   = [-500, 4900];
const WORLD_W   = WORLD_X[1]-WORLD_X[0];
const WORLD_H   = WORLD_Y[1]-WORLD_Y[0];

const trails  = {{ TAG1:[], TAG2:[] }};
const ptCount = {{ TAG1:0,  TAG2:0  }};
let showTrail = true;
let showGrid  = true;
let lastPos   = {{}};

const canvas = document.getElementById("map");
const ctx    = canvas.getContext("2d");
const wrap   = document.getElementById("canvas-wrap");
const wsBar  = document.getElementById("ws-bar");
let scale=1, offsetX=0, offsetY=0;

// Init sidebar IPs
for(const [tid,src] of Object.entries(TAG_SOURCES)){{
  const el = document.getElementById(`ip-${{tid}}`);
  if(el) el.textContent = `${{src.host}}:${{src.port}}`;
}}

function resize(){{
  canvas.width  = wrap.clientWidth;
  canvas.height = wrap.clientHeight;
  const sx=(canvas.width -MARGIN*2)/WORLD_W;
  const sy=(canvas.height-MARGIN*2)/WORLD_H;
  scale=Math.min(sx,sy);
  offsetX=MARGIN+(canvas.width -MARGIN*2-WORLD_W*scale)/2;
  offsetY=MARGIN+(canvas.height-MARGIN*2-WORLD_H*scale)/2;
}}

function toCanvas(wx,wy){{
  return {{
    x: offsetX+(wx-WORLD_X[0])*scale,
    y: canvas.height-(offsetY+(wy-WORLD_Y[0])*scale),
  }};
}}

function draw(){{
  const W=canvas.width, H=canvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle="#0d1117"; ctx.fillRect(0,0,W,H);

  // Grid
  if(showGrid){{
    ctx.strokeStyle="rgba(48,54,61,0.7)"; ctx.lineWidth=1; ctx.setLineDash([2,5]);
    for(let wx=Math.ceil(WORLD_X[0]/1000)*1000; wx<=WORLD_X[1]; wx+=1000){{
      const p=toCanvas(wx,WORLD_Y[0]),q=toCanvas(wx,WORLD_Y[1]);
      ctx.beginPath(); ctx.moveTo(p.x,p.y); ctx.lineTo(q.x,q.y); ctx.stroke();
    }}
    for(let wy=Math.ceil(WORLD_Y[0]/1000)*1000; wy<=WORLD_Y[1]; wy+=1000){{
      const p=toCanvas(WORLD_X[0],wy),q=toCanvas(WORLD_X[1],wy);
      ctx.beginPath(); ctx.moveTo(p.x,p.y); ctx.lineTo(q.x,q.y); ctx.stroke();
    }}
    ctx.setLineDash([]);
  }}

  // Axis labels
  ctx.fillStyle="#484f58"; ctx.font="10px Consolas"; ctx.textAlign="center";
  for(let wx=0; wx<=WORLD_X[1]; wx+=2000){{
    const p=toCanvas(wx,WORLD_Y[0]);
    ctx.fillText((wx/1000).toFixed(0)+"m", p.x, p.y+18);
  }}
  ctx.textAlign="right";
  for(let wy=0; wy<=WORLD_Y[1]; wy+=1000){{
    const p=toCanvas(WORLD_X[0],wy);
    ctx.fillText((wy/1000).toFixed(0)+"m", p.x-8, p.y+4);
  }}
  ctx.textAlign="left";

  // Trails
  if(showTrail){{
    for(const [tid,pts] of Object.entries(trails)){{
      if(pts.length<2) continue;
      const [r,g,b]=TAG_STYLE[tid].color;
      ctx.beginPath();
      ctx.strokeStyle=`rgba(${{r}},${{g}},${{b}},0.45)`;
      ctx.lineWidth=2; ctx.lineJoin="round";
      const p0=toCanvas(pts[0][0],pts[0][1]);
      ctx.moveTo(p0.x,p0.y);
      for(let i=1;i<pts.length;i++){{
        const p=toCanvas(pts[i][0],pts[i][1]);
        ctx.lineTo(p.x,p.y);
      }}
      ctx.stroke();
    }}
  }}

  // Anchors
  for(const a of ANCHORS){{
    const p=toCanvas(a.x,a.y);
    ctx.fillStyle="rgba(31,111,235,0.12)";
    ctx.strokeStyle="#1f6feb"; ctx.lineWidth=2;
    ctx.beginPath(); ctx.rect(p.x-9,p.y-9,18,18); ctx.fill(); ctx.stroke();
    ctx.fillStyle="#58a6ff"; ctx.font="11px Consolas"; ctx.textAlign="left";
    ctx.fillText(`${{a.label}} (${{a.x}},${{a.y}})`, p.x+13, p.y+4);
  }}

  // Tags
  for(const [tid,pts] of Object.entries(trails)){{
    if(pts.length===0) continue;
    const last=pts[pts.length-1];
    const p=toCanvas(last[0],last[1]);
    const [r,g,b]=TAG_STYLE[tid].color;
    // glow
    const grd=ctx.createRadialGradient(p.x,p.y,2,p.x,p.y,20);
    grd.addColorStop(0,`rgba(${{r}},${{g}},${{b}},0.35)`);
    grd.addColorStop(1,`rgba(${{r}},${{g}},${{b}},0)`);
    ctx.beginPath(); ctx.arc(p.x,p.y,20,0,Math.PI*2);
    ctx.fillStyle=grd; ctx.fill();
    // dot
    ctx.beginPath(); ctx.arc(p.x,p.y,8,0,Math.PI*2);
    ctx.fillStyle=`rgba(${{r}},${{g}},${{b}},0.9)`;
    ctx.strokeStyle=`rgb(${{r}},${{g}},${{b}})`; ctx.lineWidth=2;
    ctx.fill(); ctx.stroke();
    // label
    ctx.fillStyle=`rgb(${{r}},${{g}},${{b}})`; ctx.font="bold 11px Consolas";
    ctx.textAlign="center";
    ctx.fillText(TAG_STYLE[tid].name, p.x, p.y-18);
    ctx.fillStyle="#8b949e"; ctx.font="10px Consolas";
    ctx.fillText(`(${{last[0].toFixed(0)}}, ${{last[1].toFixed(0)}})`, p.x, p.y-6);
    ctx.textAlign="left";
  }}
}}

// WebSocket
function initWS(){{
  wsBar.textContent="WS: connecting…"; wsBar.style.color="#484f58";
  const ws=new WebSocket(WS_URL);
  ws.onopen=()=>{{ wsBar.textContent="WS: connected ✓"; wsBar.style.color="#3fb950"; }};
  ws.onclose=()=>{{
    wsBar.textContent="WS: disconnected — retry 3s"; wsBar.style.color="#f85149";
    setTimeout(initWS,3000);
  }};
  ws.onerror=()=>{{ wsBar.textContent="WS: error"; wsBar.style.color="#f85149"; }};
  ws.onmessage=(e)=>{{
    let msg; try{{ msg=JSON.parse(e.data); }}catch{{ return; }}

    // Connection event
    if(msg.event){{
      const dot=document.getElementById(`dot-${{msg.tag}}`);
      if(dot) dot.className="dot "+msg.event;
      return;
    }}

    const {{tag,x,y,p_onhand}}=msg;
    if(!TAG_STYLE[tag]||x==null) return;

    // Trail
    trails[tag].push([x,y]);
    if(trails[tag].length>TRAIL_MAX) trails[tag].shift();
    ptCount[tag]++;
    lastPos[tag]={{x,y}};

    // Coord
    const coord=document.getElementById(`coord-${{tag}}`);
    if(coord) coord.textContent=`X: ${{x.toFixed(0).padStart(7)}} mm\nY: ${{y.toFixed(0).padStart(7)}} mm`;

    // P(OnHand) bar
    if(p_onhand!=null){{
      const probEl=document.getElementById(`prob-${{tag}}`);
      const probLbl=document.getElementById(`prob-label-${{tag}}`);
      const probBar=document.getElementById(`prob-bar-${{tag}}`);
      if(probEl) probEl.style.display="block";
      if(probLbl) probLbl.textContent=`OnHand: ${{(p_onhand*100).toFixed(1)}}%`;
      if(probBar){{
        probBar.style.width=`${{(p_onhand*100).toFixed(1)}}%`;
        const pct=p_onhand;
        probBar.style.background=pct>=0.5?"#3fb950":"#f85149";
      }}
    }}

    // Stats
    document.getElementById("stats").textContent=
      `TAG1: ${{ptCount.TAG1}} pts\nTAG2: ${{ptCount.TAG2}} pts`;

    draw();
  }};
}}

// Controls
document.getElementById("btn-clear").onclick=()=>{{
  trails.TAG1=[]; trails.TAG2=[];
  ptCount.TAG1=0; ptCount.TAG2=0;
  draw();
}};
document.getElementById("btn-trail").onclick=function(){{
  showTrail=!showTrail;
  this.textContent=showTrail?"Trail: ON":"Trail: OFF";
  draw();
}};
document.getElementById("btn-grid").onclick=function(){{
  showGrid=!showGrid;
  this.textContent=showGrid?"Grid: ON":"Grid: OFF";
  draw();
}};

window.addEventListener("resize",()=>{{ resize(); draw(); }});
resize(); draw(); initWS();
</script>
</body>
</html>"""

# ══════════════════════════════════════════
#  HTTP SERVER
# ══════════════════════════════════════════
_html_cache = None

class HTMLHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(_html_cache.encode("utf-8"))
    def log_message(self,*a): pass

def start_http():
    srv = http.server.HTTPServer(("0.0.0.0", HTTP_PORT), HTMLHandler)
    srv.serve_forever()

# ══════════════════════════════════════════
#  TCP RECEIVER
# ══════════════════════════════════════════
def tcp_receiver(tag_id, host, port, queue):
    while True:
        try:
            queue.put({"event":"connecting","tag":tag_id})
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((host, port))
            sock.settimeout(None)
            queue.put({"event":"connected","tag":tag_id})
            buf = ""
            while True:
                data = sock.recv(4096).decode("utf-8", errors="replace")
                if not data: break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line: continue
                    try:
                        queue.put(json.loads(line))
                    except Exception:
                        pass
        except Exception as e:
            queue.put({"event":"disconnected","tag":tag_id,"msg":str(e)})
        time.sleep(3)

# ══════════════════════════════════════════
#  WEBSOCKET SERVER
# ══════════════════════════════════════════
clients    = set()
data_queue = Queue()

async def ws_handler(ws):
    clients.add(ws)
    try: await ws.wait_closed()
    finally: clients.discard(ws)

async def broadcast_loop():
    loop = asyncio.get_event_loop()
    while True:
        msg = await loop.run_in_executor(None, data_queue.get)
        if clients:
            payload = json.dumps(msg)
            await asyncio.gather(
                *[c.send(payload) for c in list(clients)],
                return_exceptions=True
            )

async def ws_main():
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        await broadcast_loop()

# ══════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════
if __name__ == "__main__":
    _html_cache = build_html(WS_PORT, ANCHORS)

    ips = get_ips()
    print("=" * 45)
    print("  UWB Web Tracking Server")
    print("=" * 45)
    print(f"  [LOCAL]     http://{ips['local']}:{HTTP_PORT}")
    if ips["tailscale"]:
        print(f"  [TAILSCALE] http://{ips['tailscale']}:{HTTP_PORT}")
    else:
        print(f"  [TAILSCALE] không tìm thấy IP 100.x.x.x")
    print(f"  [WS]        ws://0.0.0.0:{WS_PORT}")
    print("=" * 45)

    for tid, src in TAG_SOURCES.items():
        threading.Thread(
            target=tcp_receiver,
            args=(tid, src["host"], src["port"], data_queue),
            daemon=True
        ).start()
        print(f"  [TCP] {tid} <- {src['host']}:{src['port']}")

    threading.Thread(target=start_http, daemon=True).start()

    time.sleep(0.5)
    webbrowser.open(f"http://localhost:{HTTP_PORT}")

    asyncio.run(ws_main())
