import json,pathlib,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
from matplotlib.lines import Line2D
R=pathlib.Path('/home/rob/table_clean_ws/.agent_tasks');O=pathlib.Path(__file__).parent
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
def folder(prefix,round): return next(R.glob(prefix+'*'))/'output'/f'round_{round:04d}'
def draw(ax,p,full=False):
 im=plt.imread(p/'camera_rgb.jpg');ax.imshow(im)
 c=json.load(open(p/'effective_vision_config.json'));x,y,x1,y1=c['inference_roi'];ax.add_patch(Rectangle((x,y),x1-x,y1-y,fill=False,edgecolor='#ff3636',lw=2))
 s=json.load(open(p/'scene.json'));tr=s.get('tray',{})
 for key,color in [('source_polygon_px','#ffd500'),('tray_outer_polygon_px','#00e9ff')]:
  if tr.get(key):ax.add_patch(Polygon(tr[key],fill=False,edgecolor=color,lw=1.8))
 for obj in s.get('objects',[]):
  b=obj.get('pixel_bbox')
  if b:ax.add_patch(Rectangle((b[0],b[1]),b[2]-b[0],b[3]-b[1],fill=False,edgecolor='#35ff74',lw=1.3))
 if not full:ax.set_xlim(310,970);ax.set_ylim(620,365)
 ax.set_xlabel('原图 x（像素）');ax.set_ylabel('原图 y（像素）');return s
legend=[Line2D([0],[0],color=c,lw=2,label=l) for c,l in [('#ff3636','实际检测 ROI'),('#ffd500','当帧取物区投影'),('#00e9ff','当帧托盘外框投影'),('#35ff74','已检测物体框')]]
fig,ax=plt.subplots(figsize=(12.8,8.8));draw(ax,folder('2b1fe428',1),True);ax.set_title('最新任务初始全画面：相机能拍到的范围，大于实际送入检测器的 ROI');fig.legend(handles=legend,loc='lower center',ncol=4);fig.tight_layout(rect=[0,.05,1,1]);fig.savefig(O/'full_frame_roi.png',dpi=150);plt.close(fig)
cases=[('0a9d4cf0',1,'① 10/28 18:14 任务 / 第1轮：投影右端927.2，贴近边界'),('0a9d4cf0',2,'② 同任务 / 第2轮：区域越界报错，但两只碗仍检出'),('897fe062',1,'③ 10/30 18:00 任务 / 第1轮：两只待抓碗均在ROI内'),('897fe062',9,'④ 同任务 / 第9轮：两碗已入盘，区域越界阻止确认'),('9fd45b64',2,'⑤ 10/30 18:02 复核：ROI右端950，投影右端929.5'),('80d8f547',6,'⑥ 10/30 15:41 任务 / 末轮：旧ROI也有完成记录'),('2b1fe428',1,'⑦ 10/30 19:54 最新任务 / 第1轮：两只碗均在ROI内'),('2b1fe428',5,'⑧ 最新任务 / 第5轮：两碗入盘，自动完成确认')]
fig,axes=plt.subplots(4,2,figsize=(16,14))
for ax,(prefix,r,title) in zip(axes.flat,cases):
 s=draw(ax,folder(prefix,r));ax.set_title(title,fontsize=12)
 if s.get('tray',{}).get('status')=='invalid':ax.text(.02,.03,'失败帧未保存投影坐标，不借用邻帧绘制黄线',transform=ax.transAxes,color='white',fontsize=10,bbox=dict(facecolor='black',alpha=.7))
fig.legend(handles=legend,loc='lower center',ncol=4,fontsize=12);fig.suptitle('近期实机图像与ROI对照（坐标与轮廓均取自该轮快照；日期时间为任务结果时间）',fontsize=17);fig.tight_layout(rect=[0,.04,1,.97]);fig.savefig(O/'recent_roi_comparison.png',dpi=150);plt.close(fig)
# Complete task index, retaining preview failures as well as execution results.
import csv,datetime
rows=[]
for p in sorted(R.glob('*/output/result.json'),key=lambda p:p.stat().st_mtime):
 d=json.load(open(p));fails=[r.get('observation_id','').split('/')[-1] for r in d.get('rounds',[]) if r.get('error')=='source_outside_detection_view'];rows.append({'result_time_Asia_Shanghai':datetime.datetime.fromtimestamp(p.stat().st_mtime,datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'task_id':p.parts[-3],'mode':d.get('mode'),'outcome':d.get('outcome'),'error':d.get('error'),'roi':d.get('pick_region'),'roi_failure_rounds':','.join(fails),'result_path':str(p)})
with open(O/'task_index.csv','w',encoding='utf-8-sig',newline='') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
print('created figures and',len(rows),'task records')
