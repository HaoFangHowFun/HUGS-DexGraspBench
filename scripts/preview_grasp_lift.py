"""Interactive playback of recorded physics; original evaluation files unchanged."""
import argparse,queue,sys,time
from pathlib import Path
import mujoco,numpy as np
from omegaconf import OmegaConf
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from task.eval_func.tabletop_mocap import tabletopMocapEval
from util.viewer_util import MjviserDebugViewer,normalize_debug_viewer_config

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bench-dir',type=Path,required=True);p.add_argument('--port',type=int,default=8085);p.add_argument('--check',action='store_true');a=p.parse_args();bench=a.bench_dir.resolve()
 cfg=OmegaConf.load(bench/'diagnostics/resolved_config.yaml');cfg.hand.xml_path=str((ROOT/cfg.hand.xml_path).resolve());cfg.task.debug_viewer=False;cfg.task.debug_render=False;cfg.task.viewer.host='127.0.0.1';cfg.task.viewer.port=a.port
 records=[];original=mujoco.mj_step;display=None
 for path in sorted((bench/'graspdata').rglob('*_grasp.npy'),key=lambda p:int(p.stem.split('_')[0])):
  runner=tabletopMocapEval(str(path),cfg);ho=runner.mj_ho
  if display is None:display=ho
  assert ho.model.nq==display.model.nq and ho.model.ngeom==display.model.ngeom
  frames=[];steps=0
  def capture():
   frames.append({**{key:getattr(ho.data,key).copy() for key in ['qpos','qvel','mocap_pos','mocap_quat','ctrl']},'time':float(ho.data.time)})
  capture()
  def step(model,data,*args,**kw):
   nonlocal steps
   result=original(model,data,*args,**kw)
   if model is ho.model:
    steps+=1
    if steps%5==0:capture()
   return result
  mujoco.mj_step=step
  try:success,pos,angle=runner._eval_simulate_under_extforce()
  finally:mujoco.mj_step=original
  saved=np.load(bench/'evaluation'/path.relative_to(bench/'graspdata'),allow_pickle=True).item()
  assert bool(success)==bool(saved['succ_flag']) and np.isclose(pos,saved['delta_pos'],atol=1e-6),path
  records.append(dict(candidate=int(path.stem.split('_')[0]),frames=frames,success=bool(success),pos=float(pos),angle=float(angle)))
  print('Recorded',records[-1]['candidate'],steps,'steps; success',bool(success),flush=True)
 if a.check:print('All replays match saved results; no outputs overwritten.');return
 assert records
 viewer=MjviserDebugViewer(display.model,display.data,normalize_debug_viewer_config(cfg.task.viewer),0.);server=viewer.server;events=queue.SimpleQueue();index=0;frame=0;playing=False
 with server.gui.add_folder('Grasp lift replay'):
  candidate=server.gui.add_dropdown('Candidate',options=[str(r['candidate']) for r in records],initial_value=str(records[0]['candidate']))
  play=server.gui.add_button('Play / Pause');restart=server.gui.add_button('Replay from start');speed=server.gui.add_dropdown('Speed',options=['0.25x','0.5x','1x','2x'],initial_value='0.5x');timeline=server.gui.add_slider('Time (s)',min=0.,max=3.,step=.02,initial_value=0.);info=server.gui.add_markdown('')
  server.gui.add_markdown('**0–1 s close · 1–2 s squeeze · 2–3 s lift.** Recorded MuJoCo physics. Drag the timeline to inspect any frame.')
 @candidate.on_update
 def select(e):
  if e.client is not None:events.put(('candidate',e.target.value))
 @timeline.on_update
 def scrub(e):
  if e.client is not None:events.put(('time',float(e.target.value)))
 play.on_click(lambda e:events.put(('play',None)));restart.on_click(lambda e:events.put(('restart',None)))
 @server.on_client_connect
 def connect(client):
  center=(display.get_obj_pose()[:3]+display.data.qpos[:3])/2;client.camera.look_at=center;client.camera.position=center+np.array([.35,-.35,.25])
 def show():
  record=records[index];state=record['frames'][frame]
  for key in ['qpos','qvel','mocap_pos','mocap_quat','ctrl']:getattr(display.data,key)[:]=state[key]
  display.data.time=state['time'];mujoco.mj_forward(display.model,display.data);viewer.sync();timeline.value=round(frame*.02,2)
  stage='Close' if frame<50 else 'Squeeze' if frame<100 else 'Lift'
  info.content=f"**Candidate {record['candidate']} — {'PASS' if record['success'] else 'FAIL'}**\n\n{stage}: {frame*.02:.2f} s · {'Playing' if playing else 'Paused'}\n\nFinal position error: {record['pos']*1000:.1f} mm; rotation error: {record['angle']:.1f}°"
 show();print(f'Open http://localhost:{a.port}; press Play. Default half speed.',flush=True);tick=time.monotonic()
 try:
  while True:
   dirty=False
   while not events.empty():
    kind,value=events.get()
    if kind=='candidate':index=next(i for i,r in enumerate(records) if r['candidate']==int(value));frame=0;playing=False
    elif kind=='time':frame=min(round(value/.02),len(records[index]['frames'])-1);playing=False
    elif kind=='restart':frame=0;playing=True
    elif kind=='play':
     if frame==len(records[index]['frames'])-1:frame=0
     playing=not playing
    tick=time.monotonic();dirty=True
   now=time.monotonic()
   if playing and now-tick>=.02/float(speed.value[:-1]):
    frame=min(frame+1,len(records[index]['frames'])-1);playing=frame<len(records[index]['frames'])-1;tick=now;dirty=True
   if dirty:show()
   time.sleep(.005)
 except KeyboardInterrupt:pass
 finally:viewer.close()
if __name__=='__main__':main()
