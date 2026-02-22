/**
 * @file portal_handler.c
 * @brief HTTP handler — serves the embedded HTML config portal
 *
 * Single-page application with tabbed navigation:
 *   - Status: system health, link state, heap, uptime
 *   - Logs: SD card file list, download, delete
 *   - Gauges: slot configuration with PID/unit dropdowns
 *   - Settings: backlight, splash duration, WiFi config
 *
 * The HTML/CSS/JS is embedded as a C string constant — no SPIFFS needed.
 * All data operations use fetch() against the JSON API endpoints.
 *
 * GET /           → HTML portal
 * GET /favicon.ico → 204 No Content (suppresses browser 404)
 */

#include "wifi_internal.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "http_portal";

// ============================================================================
// Embedded HTML Portal
// ============================================================================

static const char PORTAL_HTML[] =
"<!DOCTYPE html>"
"<html lang='en'><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>CAN Display</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:system-ui,-apple-system,sans-serif;background:#0f0f1a;color:#e0e0e0;"
"max-width:600px;margin:0 auto;padding:0 0 80px}"
"header{background:#16213e;padding:14px 16px;display:flex;align-items:center;gap:10px;"
"position:sticky;top:0;z-index:10;border-bottom:2px solid #4fc3f7}"
"header h1{font-size:1.15em;color:#4fc3f7;font-weight:600}"
"nav{display:flex;background:#16213e;border-bottom:1px solid #1a2744;overflow-x:auto}"
"nav button{flex:1;padding:10px 6px;border:none;background:none;color:#888;"
"font-size:.85em;cursor:pointer;border-bottom:2px solid transparent;white-space:nowrap}"
"nav button.active{color:#4fc3f7;border-bottom-color:#4fc3f7}"
".page{display:none;padding:12px 14px}"
".page.active{display:block}"
".card{background:#16213e;border-radius:8px;padding:14px;margin:10px 0}"
".card h3{font-size:.85em;color:#888;margin-bottom:6px;text-transform:uppercase;"
"letter-spacing:.5px}"
".val{font-size:1.2em;font-weight:700}"
".row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;"
"border-bottom:1px solid #1a2744}"
".row:last-child{border-bottom:none}"
".ok{color:#66bb6a}.warn{color:#ffa726}.err{color:#ef5350}"
"button,.btn{background:#4fc3f7;color:#000;border:none;padding:8px 18px;"
"border-radius:6px;font-size:.9em;cursor:pointer;font-weight:600}"
"button:active,.btn:active{background:#039be5}"
".btn-sm{padding:5px 12px;font-size:.8em}"
".btn-danger{background:#ef5350;color:#fff}"
".btn-danger:active{background:#c62828}"
"input,select{background:#0f0f1a;border:1px solid #2a3a5c;color:#e0e0e0;"
"padding:8px 10px;border-radius:6px;font-size:.9em;width:100%}"
"input:focus,select:focus{outline:none;border-color:#4fc3f7}"
"label{display:block;font-size:.8em;color:#888;margin:10px 0 4px}"
".slider-row{display:flex;align-items:center;gap:10px}"
".slider-row input[type=range]{flex:1}"
".slider-row span{min-width:36px;text-align:right;font-weight:700}"
".log-item{display:flex;justify-content:space-between;align-items:center;"
"padding:10px 0;border-bottom:1px solid #1a2744}"
".log-item:last-child{border-bottom:none}"
".log-name{font-weight:600;word-break:break-all}"
".log-size{color:#888;font-size:.85em}"
".log-actions{display:flex;gap:6px}"
".gauge-slot{padding:10px 0;border-bottom:1px solid #1a2744}"
".gauge-slot:last-child{border-bottom:none}"
".gauge-header{display:flex;justify-content:space-between;align-items:center}"
".gauge-id{color:#4fc3f7;font-weight:700;font-size:.85em}"
".gauge-val{font-weight:600}"
".empty{color:#555;font-style:italic}"
".toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);"
"background:#333;color:#fff;padding:10px 24px;border-radius:8px;"
"font-size:.9em;opacity:0;transition:opacity .3s;z-index:100;pointer-events:none}"
".toast.show{opacity:1}"
"input[type=checkbox]{accent-color:#4fc3f7;width:18px;height:18px;cursor:pointer}"
".pid-tbl{width:100%;border-collapse:collapse}"
".pid-tbl th{color:#888;font-size:.8em;text-align:left;padding:6px 4px;border-bottom:1px solid #2a3a5c}"
".pid-tbl td{padding:6px 4px;border-bottom:1px solid #1a2744}"
".pid-hex{color:#4fc3f7;font-family:monospace;font-size:.9em}"
".pid-unit{color:#888;font-size:.85em}"
".alert-in{width:52px;padding:4px;font-size:.8em;text-align:center}"
".pid-toolbar{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;gap:8px}"
".pid-toolbar span{color:#888;font-size:.85em}"
".dtc-tbl{width:100%;border-collapse:collapse}"
".dtc-tbl th{color:#888;font-size:.8em;text-align:left;padding:6px 4px;border-bottom:1px solid #2a3a5c}"
".dtc-tbl td{padding:6px 4px;border-bottom:1px solid #1a2744}"
".dtc-code{color:#ef5350;font-family:monospace;font-size:.95em;font-weight:700}"
".dtc-sys{font-size:.85em}"
".dtc-none{color:#66bb6a;font-weight:600}"
"@media(max-width:400px){.card{padding:10px}header h1{font-size:1em}}"
"</style></head><body>"
"<header><h1>&#x1F697; CAN Display</h1></header>"

/* ── Navigation ── */
"<nav>"
"<button class='active' onclick='tab(\"status\")'>Status</button>"
"<button onclick='tab(\"vehicle\")'>Vehicle</button>"
"<button onclick='tab(\"logs\")'>Logs</button>"
"<button onclick='tab(\"pids\")'>PIDs</button>"
"<button onclick='tab(\"workshop\")'>Workshop</button>"
"<button onclick='tab(\"settings\")'>Settings</button>"
"</nav>"

/* ── Status Page ── */
"<div id='p-status' class='page active'>"
"<div class='card'><h3>&#x1F551; Time</h3><div id='time-info'>Syncing...</div></div>\n"
"<div class='card'><h3>System</h3><div id='sys-info'>Loading...</div></div>"
"<div class='card'><h3>CAN Link</h3><div id='link-info'></div></div>"
"<div class='card'><h3>Memory</h3><div id='mem-info'></div></div>"
"</div>"

/* ── Vehicle Page ── */
"<div id='p-vehicle' class='page'>"
"<div class='card'><h3>&#x1F697; Vehicle Info</h3>"
"<div id='veh-info'>Loading...</div>"
"<button class='btn-sm' style='margin-top:8px' id='scan-btn' onclick='scanVehicle()'>Scan Vehicle</button>"
"</div>"
"<div class='card'><h3>&#x26A0; Diagnostic Trouble Codes</h3>"
"<div id='dtc-info'>No DTC data — scan or read first.</div>"
"<div style='margin-top:8px;display:flex;gap:6px'>"
"<button class='btn-sm' id='dtc-read-btn' onclick='readDtcs()'>Read DTCs</button>"
"<button class='btn-sm btn-danger' id='dtc-clear-btn' onclick='clearDtcs()'>Clear DTCs</button>"
"</div></div>"
"</div>"

/* ── Logs Page ── */
"<div id='p-logs' class='page'>"
"<div class='card'><h3>SD Card</h3><div id='sd-info'></div></div>"
"<div class='card'><h3>Log Files</h3><div id='log-list'>Loading...</div></div>"
"</div>"

/* ── PIDs Page ── */
"<div id='p-pids' class='page'>"
"<div class='card'><h3>Supported PIDs</h3>"
"<div class='pid-toolbar'>"
"<span id='pid-count'></span>"
"<div style='display:flex;gap:6px'>"
"<button class='btn-sm' onclick='pidSelectAll(true)'>All</button>"
"<button class='btn-sm' onclick='pidSelectAll(false)'>None</button>"
"<button class='btn-sm' onclick='applyPoll()'>Apply</button>"
"<button class='btn-sm' onclick='saveAlerts()'>Alerts</button>"
"</div></div>"
"<div id='pid-list' style='overflow-x:auto'>Loading...</div>"
"</div></div>"

/* ── Workshop Page ── */
"<div id='p-workshop' class='page'>"
"<div class='card'><h3>Workshop</h3>"
"<p style='color:#888;font-size:.9em;margin-bottom:12px'>Build custom data channels, math expressions, and virtual I/O. Coming soon.</p>"
"</div>"
"<div class='card'><h3>&#x2699; Math Channels</h3>"
"<p style='color:#666;font-size:.85em'>Define computed values from PID data — averages, ratios, delta-T, min/max tracking.</p>"
"<div style='opacity:.5;pointer-events:none;margin-top:10px'>"
"<div class='row'><span class='gauge-id'>CH-1</span><span class='empty'>AFR = (PID 0x44 &times; 14.7)</span></div>"
"<div class='row'><span class='gauge-id'>CH-2</span><span class='empty'>Fuel Rate = f(0x5E, 0x0D)</span></div>"
"<div class='row'><span class='gauge-id'>CH-3</span><span class='empty'>&#x0394;Coolant = dT(0x05)</span></div>"
"</div></div>"
"<div class='card'><h3>&#x1F4E1; Virtual Inputs</h3>"
"<p style='color:#666;font-size:.85em'>Map external sensors, analog inputs, or derived signals as virtual PIDs.</p>"
"<div style='opacity:.5;pointer-events:none;margin-top:10px'>"
"<div class='row'><span class='gauge-id'>VIRT-1</span><span class='empty'>Wideband O2 (ADC ch0)</span></div>"
"<div class='row'><span class='gauge-id'>VIRT-2</span><span class='empty'>EGT (thermocouple)</span></div>"
"</div></div>"
"<div class='card'><h3>&#x1F4CA; Trip Accumulators</h3>"
"<p style='color:#666;font-size:.85em'>Track running totals — fuel used, distance, idle time, peak values per trip.</p>"
"<div style='opacity:.5;pointer-events:none;margin-top:10px'>"
"<div class='row'><span class='gauge-id'>TRIP-1</span><span class='empty'>Fuel consumed (gal)</span></div>"
"<div class='row'><span class='gauge-id'>TRIP-2</span><span class='empty'>Max RPM this trip</span></div>"
"</div></div>"
"<div class='card'><h3>&#x26A0; Alerts &amp; Triggers</h3>"
"<p style='color:#666;font-size:.85em'>Set thresholds that trigger on-screen warnings, buzzer, or logger events.</p>"
"<div style='opacity:.5;pointer-events:none;margin-top:10px'>"
"<div class='row'><span class='gauge-id'>ALT-1</span><span class='empty'>Coolant &gt; 230&deg;F → warn</span></div>"
"<div class='row'><span class='gauge-id'>ALT-2</span><span class='empty'>Oil PSI &lt; 15 → buzzer</span></div>"
"</div></div>"
"</div>"

/* ── Settings Page ── */
"<div id='p-settings' class='page'>"
"<div class='card'><h3>Backlight</h3>"
"<div class='slider-row'>"
"<input type='range' id='bl-slider' min='0' max='100' oninput='blPreview(this.value)'>"
"<span id='bl-val'>-</span>"
"</div>"
"<button class='btn-sm' style='margin-top:8px' onclick='blSave()'>Apply</button>"
"</div>"
"<div class='card'><h3>Boot Splash</h3>"
"<div class='slider-row'>"
"<input type='range' id='sp-slider' min='0' max='10000' step='500' oninput='spPreview(this.value)'>"
"<span id='sp-val'>-</span>"
"</div>"
"<button class='btn-sm' style='margin-top:8px' onclick='spSave()'>Apply</button>"
"</div>"
"<div class='card'><h3>WiFi AP</h3>"
"<label>SSID Override (empty = auto)</label>"
"<input id='wifi-ssid' placeholder='CAN_XXXX'>"
"<label>Password Override (empty = random)</label>"
"<input id='wifi-pass' placeholder='Random each start'>"
"<label>Channel (1-13)</label>"
"<input id='wifi-ch' type='number' min='1' max='13' value='1'>"
"<div style='margin-top:10px'>"
"<button class='btn-sm' onclick='wifiSave()'>Save WiFi Settings</button>"
"</div>"
"</div>"
"<div class='card'><h3>Polling</h3>"
"<div class='row'><span>Auto-Poll on Boot</span>"
"<input type='checkbox' id='ap-toggle' onchange='apToggle(this.checked)'>"
"</div>"
"<p style='color:#666;font-size:.8em;margin-top:6px'>When enabled, polling starts automatically after scan completes using your saved PID selection.</p>"
"</div>"
"</div>"

/* ── Toast ── */
"<div id='toast' class='toast'></div>"

"<script>"
/* ── Utility ── */
"const $=s=>document.getElementById(s);"
"function toast(m){let t=$('toast');t.textContent=m;t.classList.add('show');"
"setTimeout(()=>t.classList.remove('show'),2000)}"
"function fmt(s){let h=Math.floor(s/3600),m=Math.floor(s%3600/60),sc=s%60;"
"return h+'h '+m+'m '+sc+'s'}"
"function fmtBytes(b){if(b>1073741824)return(b/1073741824).toFixed(1)+'GB';"
"if(b>1048576)return(b/1048576).toFixed(1)+'MB';"
"if(b>1024)return(b/1024).toFixed(0)+'KB';return b+'B'}"

/* ── Tab Switching ── */
"function tab(id){"
"document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));"
"document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));"
"$('p-'+id).classList.add('active');"
"event.target.classList.add('active');"
"if(id==='status'){loadStatus()}"
"if(id==='vehicle'){loadVehicle();loadDtcs()}"
"if(id==='logs'){loadSD();loadLogs()}"
"if(id==='pids')loadPids();"
"if(id==='workshop')loadWorkshop();"
"if(id==='settings')loadSettings();"
"}"

/* ── Status Page ── */
"function loadStatus(){"
"fetch('/api/status').then(r=>r.json()).then(d=>{"
"$('sys-info').innerHTML="
"'<div class=\"row\"><span>Uptime</span><span class=\"val\">'+fmt(d.uptime_s)+'</span></div>'"
"+'<div class=\"row\"><span>WiFi Clients</span><span class=\"val\">'+d.wifi_clients+'</span></div>'"
"+'<div class=\"row\"><span>SD Card</span><span class=\"val '+(d.sd_mounted?'ok':'warn')+'\">'+(d.sd_mounted?'Mounted':'None')+'</span></div>'"
"+'<div class=\"row\"><span>Logger</span><span class=\"val\">'+d.logger+'</span></div>'"
"+'<div class=\"row\"><span>Brightness</span><span class=\"val\">'+d.brightness+'%</span></div>';"
"$('link-info').innerHTML="
"'<div class=\"row\"><span>State</span><span class=\"val '+(d.link==='connected'?'ok':'warn')+'\">'+"
"d.link+'</span></div>'"
"+'<div class=\"row\"><span>RX Frames</span><span>'+d.rx_frames+'</span></div>'"
"+'<div class=\"row\"><span>TX Frames</span><span>'+d.tx_frames+'</span></div>'"
"+'<div class=\"row\"><span>PID Updates</span><span>'+d.pid_updates+'</span></div>'"
"+'<div class=\"row\"><span>RX Errors</span><span class=\"'+(d.rx_errors>0?'err':'')+'\">'+"
"d.rx_errors+'</span></div>';"
"$('mem-info').innerHTML="
"'<div class=\"row\"><span>Heap Free</span><span>'+fmtBytes(d.heap_free)+'</span></div>'"
"+'<div class=\"row\"><span>Heap Min</span><span>'+fmtBytes(d.heap_min)+'</span></div>'"
"+'<div class=\"row\"><span>Internal</span><span>'+fmtBytes(d.heap_internal)+'</span></div>'"
"+'<div class=\"row\"><span>PSRAM</span><span>'+fmtBytes(d.heap_psram)+'</span></div>';"
"}).catch(e=>$('sys-info').textContent='Error: '+e)}"

/* ── Logs Page ── */
"function loadSD(){"
"fetch('/api/sd').then(r=>r.json()).then(d=>{"
"$('sd-info').innerHTML="
"'<div class=\"row\"><span>Mounted</span><span class=\"'+(d.mounted?'ok':'warn')+'\">'+"
"(d.mounted?'Yes':'No')+'</span></div>'"
"+'<div class=\"row\"><span>Total</span><span>'+fmtBytes(d.total_bytes)+'</span></div>'"
"+'<div class=\"row\"><span>Free</span><span class=\"'+(d.low_space?'warn':'')+'\">'+"
"fmtBytes(d.free_bytes)+(d.low_space?' LOW!':'')+'</span></div>';"
"}).catch(e=>$('sd-info').textContent='Error')}"

"function loadLogs(){"
"fetch('/api/logs').then(r=>r.json()).then(files=>{"
"if(files.error){$('log-list').textContent=files.error;return}"
"if(!files.length){$('log-list').innerHTML='<span class=\"empty\">No log files</span>';return}"
"files.sort((a,b)=>a.name.localeCompare(b.name,undefined,{numeric:true}));"
"let h='';"
"files.forEach(f=>{"
"h+='<div class=\"log-item\">';"
"h+='<div><div class=\"log-name\">'+f.name+'</div><div class=\"log-size\">'+fmtBytes(f.size)+'</div></div>';"
"h+='<div class=\"log-actions\">';"
"h+='<button class=\"btn-sm\" onclick=\"dlLog(\\''+f.name+'\\')\">&darr;</button>';"
"h+='<button class=\"btn-sm btn-danger\" onclick=\"delLog(\\''+f.name+'\\')\">&times;</button>';"
"h+='</div></div>';"
"});"
"$('log-list').innerHTML=h;"
"}).catch(e=>$('log-list').textContent='Error')}"

"function dlLog(name){window.location='/api/logs/'+encodeURIComponent(name)}"
"function delLog(name){if(!confirm('Delete '+name+'?'))return;"
"fetch('/api/logs/'+encodeURIComponent(name),{method:'DELETE'})"
".then(r=>r.json()).then(d=>{toast('Deleted');loadLogs()}).catch(e=>toast('Error'))}"

/* ── PIDs Page ── */
"let polledPids=new Set();"
"function loadPids(){"
"fetch('/api/pids').then(r=>r.json()).then(d=>{"
"polledPids.clear();"
"let h='<table class=\"pid-tbl\"><tr><th>Poll</th><th>PID</th><th>Name</th><th>Unit</th>"
"<th>Warn</th><th>Crit</th><th>Max</th></tr>';"
"d.pids.forEach(p=>{"
"if(p.selected)polledPids.add(p.id);"
"h+='<tr><td style=\"text-align:center\"><input type=\"checkbox\" data-pid=\"'+p.id+'\"';"
"h+=p.selected?' checked':'';"
"h+=' onchange=\"togglePid('+p.id+',this.checked)\"></td>';"
"h+='<td class=\"pid-hex\">'+p.hex+'</td>';"
"h+='<td>'+p.name+'</td>';"
"h+='<td class=\"pid-unit\">'+(p.unit||'-')+'</td>';"
"h+='<td><input type=\"number\" class=\"alert-in\" data-pid=\"'+p.id+'\" data-f=\"w\" value=\"'+(p.warn||'')+'\" step=\"any\"></td>';"
"h+='<td><input type=\"number\" class=\"alert-in\" data-pid=\"'+p.id+'\" data-f=\"c\" value=\"'+(p.crit||'')+'\" step=\"any\"></td>';"
"h+='<td><input type=\"number\" class=\"alert-in\" data-pid=\"'+p.id+'\" data-f=\"m\" value=\"'+(p.max||'')+'\" step=\"any\"></td></tr>';"
"});"
"h+='</table>';"
"$('pid-list').innerHTML=h;"
"updPidCount(d.total);"
"}).catch(e=>$('pid-list').textContent='Error: '+e)}"

"function togglePid(id,on){"
"if(on)polledPids.add(id);else polledPids.delete(id);updPidCount()}"

"function updPidCount(total){"
"let t=total||document.querySelectorAll('.pid-tbl input').length;"
"$('pid-count').textContent=polledPids.size+' / '+t+' selected'}"

"function pidSelectAll(on){"
"document.querySelectorAll('.pid-tbl input[type=checkbox]').forEach(cb=>{"
"cb.checked=on;let id=parseInt(cb.dataset.pid);"
"if(on)polledPids.add(id);else polledPids.delete(id);});"
"updPidCount()}"

"function applyPoll(){"
"let pids=Array.from(polledPids);"
"fetch('/api/pids/poll',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({pids:pids,rate_hz:10})})"
".then(r=>r.json()).then(d=>{toast('Poll: '+d.count+' PIDs @ '+d.rate_hz+'Hz')})"
".catch(e=>toast('Error'))}"

"function saveAlerts(){"
"let a=[];"
"document.querySelectorAll('.alert-in[data-f=\"w\"]').forEach(inp=>{"
"let pid=parseInt(inp.dataset.pid);"
"let w=parseFloat(inp.value)||0;"
"let ce=document.querySelector('.alert-in[data-pid=\"'+pid+'\"][data-f=\"c\"]');"
"let me=document.querySelector('.alert-in[data-pid=\"'+pid+'\"][data-f=\"m\"]');"
"let c=ce?parseFloat(ce.value)||0:0;"
"let m=me?parseFloat(me.value)||0:0;"
"if(w||c||m)a.push({id:pid,warn:w,crit:c,max:m});"
"});"
"fetch('/api/pids/alerts',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({alerts:a})}).then(r=>r.json()).then(d=>{"
"if(d.ok)toast('Alerts saved ('+d.count+' PIDs)');else toast('Save failed');"
"}).catch(e=>toast('Error: '+e))}"

/* ── Workshop Page ── */
"function loadWorkshop(){/* stub — future: load custom channel definitions */}"

/* ── Vehicle / Scan ── */
"function loadVehicle(){"
"fetch('/api/can/vehicle').then(r=>r.json()).then(d=>{"
"let h='';"
"h+='<div class=\"row\"><span>Scan Status</span><span class=\"val '+"
"(d.status==='complete'?'ok':d.status==='scanning'?'warn':'')+'\">'+"
"d.status+'</span></div>';"
"if(d.has_info){"
"h+='<div class=\"row\"><span>VIN</span><span class=\"val\">'+d.vin+'</span></div>';"
"h+='<div class=\"row\"><span>ECUs</span><span>'+d.ecu_count+'</span></div>';"
"h+='<div class=\"row\"><span>Supported PIDs</span><span>'+d.pid_count+'</span></div>';"
"if(d.ecu_name)h+='<div class=\"row\"><span>ECU Name</span><span>'+d.ecu_name+'</span></div>';"
"if(d.cal_id)h+='<div class=\"row\"><span>Calibration ID</span><span>'+d.cal_id+'</span></div>';"
"if(d.cvn)h+='<div class=\"row\"><span>CVN</span><span>'+d.cvn+'</span></div>';"
"h+='<div class=\"row\"><span>DTCs</span><span class=\"'+(d.dtc_count>0?'warn':'')+'\">'+"
"d.dtc_count+'</span></div>';"
"h+='<div class=\"row\"><span>Check Engine (MIL)</span><span class=\"'+(d.mil_status?'crit':(d.emission_dtc_count>0?'warn':'ok'))+'\">'+ "
"(d.mil_status?'\u26A0 ON':(d.emission_dtc_count>0?'\u26A0 OFF ('+d.emission_dtc_count+' emission DTC'+(d.emission_dtc_count>1?'s':'')+')':'\u2714 OFF'))+'</span></div>';""}"
"$('veh-info').innerHTML=h;"
"if(d.status==='scanning'){"
"$('scan-btn').textContent='Scanning...';"
"$('scan-btn').disabled=true;"
"setTimeout(loadVehicle,1500);"
"}else{"
"$('scan-btn').textContent='Scan Vehicle';"
"$('scan-btn').disabled=false;}"
"}).catch(e=>$('veh-info').textContent='Error: '+e)}"

"function scanVehicle(){"
"$('scan-btn').textContent='Scanning...';$('scan-btn').disabled=true;"
"fetch('/api/can/scan',{method:'POST'}).then(r=>r.json()).then(d=>{"
"if(d.ok){toast('Scan started');setTimeout(loadVehicle,2000)}"
"else{toast('Scan failed: '+d.reason);$('scan-btn').textContent='Scan Vehicle';$('scan-btn').disabled=false;}"
"}).catch(e=>{toast('Error: '+e);$('scan-btn').textContent='Scan Vehicle';$('scan-btn').disabled=false;})}"

/* ── DTC Functions ── */
"function loadDtcs(){"
"fetch('/api/can/dtcs').then(r=>r.json()).then(d=>{"
"if(!d.has_data){$('dtc-info').innerHTML='<span class=\"empty\">No DTC data — scan or read first.</span>';return}"
"if(d.count===0){$('dtc-info').innerHTML='<span class=\"dtc-none\">&#x2714; No trouble codes found</span>';return}"
"let h='<table class=\"dtc-tbl\"><tr><th>Code</th><th>System</th><th>Type</th></tr>';"
"d.dtcs.forEach(t=>{"
"h+='<tr><td class=\"dtc-code\">'+t.code+'</td>';"
"h+='<td class=\"dtc-sys\">'+t.system+'</td>';"
"h+='<td class=\"dtc-sys\">'+t.type+'</td></tr>';});"
"h+='</table>';"
"$('dtc-info').innerHTML=h;"
"}).catch(e=>$('dtc-info').textContent='Error: '+e)}"

"function readDtcs(){"
"$('dtc-read-btn').textContent='Reading...';$('dtc-read-btn').disabled=true;"
"fetch('/api/can/dtcs/read',{method:'POST'}).then(r=>r.json()).then(d=>{"
"if(d.ok){toast('Reading DTCs...');setTimeout(()=>{loadDtcs();$('dtc-read-btn').textContent='Read DTCs';$('dtc-read-btn').disabled=false},3000)}"
"else{toast('Read failed: '+d.reason);$('dtc-read-btn').textContent='Read DTCs';$('dtc-read-btn').disabled=false;}"
"}).catch(e=>{toast('Error: '+e);$('dtc-read-btn').textContent='Read DTCs';$('dtc-read-btn').disabled=false;})}"

"function clearDtcs(){"
"if(!confirm('Clear all DTCs? This will reset the Check Engine Light.'))return;"
"$('dtc-clear-btn').textContent='Clearing...';$('dtc-clear-btn').disabled=true;"
"fetch('/api/can/dtcs/clear',{method:'POST'}).then(r=>r.json()).then(d=>{"
"if(d.ok){toast('Clearing DTCs...');setTimeout(()=>{loadDtcs();$('dtc-clear-btn').textContent='Clear DTCs';$('dtc-clear-btn').disabled=false},3000)}"
"else{toast('Clear failed: '+d.reason);$('dtc-clear-btn').textContent='Clear DTCs';$('dtc-clear-btn').disabled=false;}"
"}).catch(e=>{toast('Error: '+e);$('dtc-clear-btn').textContent='Clear DTCs';$('dtc-clear-btn').disabled=false;})}"

/* ── Settings Page ── */
"function loadSettings(){"
"fetch('/api/config/backlight').then(r=>r.json()).then(d=>{"
"$('bl-slider').value=d.brightness;$('bl-val').textContent=d.brightness+'%';"
"});"
"fetch('/api/config/splash').then(r=>r.json()).then(d=>{"
"$('sp-slider').value=d.duration_ms;"
"$('sp-val').textContent=(d.duration_ms/1000).toFixed(1)+'s';"
"});"
"fetch('/api/config/wifi').then(r=>r.json()).then(d=>{"
"$('wifi-ssid').placeholder=d.ssid;"
"});"
"fetch('/api/config/autopoll').then(r=>r.json()).then(d=>{"
"$('ap-toggle').checked=d.enabled;"
"});"
"}"

"function blPreview(v){$('bl-val').textContent=v+'%'}"
"function blSave(){let v=parseInt($('bl-slider').value);"
"fetch('/api/config/backlight',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({brightness:v})}).then(()=>toast('Brightness: '+v+'%'))}"

"function spPreview(v){$('sp-val').textContent=(v/1000).toFixed(1)+'s'}"
"function spSave(){let v=parseInt($('sp-slider').value);"
"fetch('/api/config/splash',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({duration_ms:v})}).then(()=>toast('Splash: '+(v/1000).toFixed(1)+'s'))}"

"function wifiSave(){"
"let body={};"
"let s=$('wifi-ssid').value;if(s)body.ssid=s;"
"let p=$('wifi-pass').value;if(p)body.pass=p;"
"let c=parseInt($('wifi-ch').value);if(c>=1&&c<=13)body.channel=c;"
"fetch('/api/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify(body)}).then(()=>toast('WiFi settings saved (next AP start)'))}"

"function apToggle(on){"
"fetch('/api/config/autopoll',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({enabled:on})}).then(()=>toast('Auto-poll: '+(on?'ON':'OFF')))}"

/* ── Time Sync ── */
"function syncTime(){"
"let epoch=Math.floor(Date.now()/1000);"
"let tz=new Date().getTimezoneOffset();"    /* JS returns minutes, inverted sign */
"$('time-info').innerHTML='<span style=\"color:#888\">Syncing clock...</span>';"
"fetch('/api/settime',{method:'POST',headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({epoch:epoch,tz_offset:tz})})"
".then(r=>r.json()).then(d=>{"
"if(d.ok){"
"$('time-info').innerHTML="
"'<div class=\"row\"><span>Local Time</span><span class=\"val ok\">'+d.local_time+'</span></div>'"
"+'<div class=\"row\"><span>Timezone</span><span>UTC'+(d.tz_offset>=0?'+':'')+Math.floor(d.tz_offset/60)+':'+String(Math.abs(d.tz_offset)%60).padStart(2,'0')+'</span></div>'"
"+'<div class=\"row\"><span>RTC</span><span class=\"'+(d.rtc?'ok':'warn')+'\">'+(d.rtc?'Synced':'No RTC')+'</span></div>';"
"toast('Clock synced');"
"}else{"
"$('time-info').innerHTML='<span class=\"err\">Sync failed</span>';"
"}"
"}).catch(e=>$('time-info').innerHTML='<span class=\"err\">Error: '+e+'</span>')}"

"function loadTime(){"
"fetch('/api/time').then(r=>r.json()).then(d=>{"
"let tz=d.tz_offset;"
"$('time-info').innerHTML="
"'<div class=\"row\"><span>Local Time</span><span class=\"val'+(d.valid?' ok':' warn')+'\">'+(d.valid?d.local:'Not set')+'</span></div>'"
"+'<div class=\"row\"><span>UTC</span><span>'+(d.valid?d.utc:'-')+'</span></div>'"
"+'<div class=\"row\"><span>Timezone</span><span>UTC'+(tz>=0?'+':'')+Math.floor(tz/60)+':'+String(Math.abs(tz)%60).padStart(2,'0')+'</span></div>'"
"+'<div class=\"row\"><span>RTC</span><span class=\"'+(d.rtc?'ok':'warn')+'\">'+(d.rtc?'Active':'None')+'</span></div>';"
"}).catch(e=>$('time-info').textContent='Error')}"

/* ── Init ── */
"syncTime();loadStatus();setInterval(()=>{if($('p-status').classList.contains('active'))loadStatus()},5000);"
"</script></body></html>";

// ============================================================================
// GET / — serve portal HTML
// ============================================================================

static esp_err_t portal_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_HTML, sizeof(PORTAL_HTML) - 1);
    return ESP_OK;
}

// ============================================================================
// GET /favicon.ico — suppress browser 404
// ============================================================================

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_portal = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = portal_get_handler,
};

static const httpd_uri_t uri_favicon = {
    .uri      = "/favicon.ico",
    .method   = HTTP_GET,
    .handler  = favicon_handler,
};

esp_err_t portal_handler_register(httpd_handle_t server)
{
    esp_err_t ret;

    ret = httpd_register_uri_handler(server, &uri_portal);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed GET /: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_favicon);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed GET /favicon.ico: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Registered: portal & favicon");
    return ret;
}
