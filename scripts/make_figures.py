# -*- coding: utf-8 -*-
"""결과 그림 4장 만들기 (원본 SD 로그 -> results/figures)

사용법 (저장소 맨 위 폴더에서):
    python scripts/make_figures.py --logs data/raw --pid B --out results/figures

로그 파일 이름 규칙 (data/raw 안):
    <참가자>_0kg_1*.csv, <참가자>_0kg_2*.csv, <참가자>_3kg_OFF*.csv, <참가자>_3kg_ON*.csv,
    <참가자>_6kg_OFF*.csv, <참가자>_6kg_ON*.csv
"""
import sys, os, glob, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'analysis'))
import numpy as np, h10_analyze as h
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from scipy import stats
ap = argparse.ArgumentParser()
ap.add_argument('--logs', default='data/raw'); ap.add_argument('--pid', default='B'); ap.add_argument('--out', default='results/figures')
A = ap.parse_args(); os.makedirs(A.out, exist_ok=True)
for f in fm.findSystemFonts():
    if 'NotoSansCJK-Regular' in f or 'malgun' in f.lower(): fm.fontManager.addfont(f)
FONT = 'Malgun Gothic' if any('malgun' in f.lower() for f in fm.findSystemFonts()) else 'Noto Sans CJK JP'
plt.rcParams.update({'font.family':FONT,'axes.unicode_minus':False,'font.size':11,
    'axes.edgecolor':'#c8c7c2','axes.labelcolor':'#52514e','xtick.color':'#52514e','ytick.color':'#52514e',
    'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#fcfcfb','axes.facecolor':'#fcfcfb',
    'savefig.facecolor':'#fcfcfb','grid.color':'#e6e5e1','grid.linewidth':0.8})
INK='#0b0b0b'; INK2='#52514e'; MUTE='#8a8984'
OFFC='#eb6834'; ONC='#2a78d6'; NORMC='#b9b8b3'
OUT = A.out.rstrip('/\\') + os.sep
def find(pat):
    c = sorted(glob.glob(os.path.join(A.logs, f'{A.pid}_{pat}*')))
    if not c: sys.exit(f'로그를 찾을 수 없습니다: {A.logs}/{A.pid}_{pat}*')
    return c[-1]
KEYS = {'0kg_1':'0kg_1','0kg_2':'0kg_2','3kg_OFF':'3kg_OFF','3kg_ON':'3kg_ON','6kg_OFF':'6kg_OFF','6kg_ON':'6kg_ON'}
D = {k: h.decode(find(p)) for k, p in KEYS.items()}
def split_sig(d,fs=1000.):
    Dr=d['thighDeg_RH']-d['thighDeg_LH']; a=(1/fs)/(0.04+1/fs)
    s1=s2=mid=Dr[0]; hi,lo=-1e9,1e9; state=0; ev=[]; sig=np.empty_like(Dr); last=None
    for k,x in enumerate(Dr):
        s1+=a*(x-s1); s2+=a*(s1-s2); mid+=(1/fs/3)*(s2-mid); sig[k]=s2
        hi=max(hi,s2); lo=min(lo,s2); st=1 if s2>mid else -1
        if st==1 and state==-1:
            if hi-lo>15 and last is not None:
                T=(k-last)/fs
                if 0.7<T<2.0: ev.append((k,(hi+lo)/(hi-lo),hi,-lo,T))
            last=k; hi,lo=-1e9,1e9
        state=st
    return sig,np.array(ev)
S={k:split_sig(d) for k,d in D.items()}

# ---------- Fig 1: 벌어짐 각도 시계열 ----------
def best_window(ev,L=8000):
    t=ev[:,0]; best=(0,t[0])
    for s in t:
        n=np.sum((t>=s)&(t<s+L))
        if n>best[0]: best=(n,s)
    return int(best[1])
fig,ax=plt.subplots(2,2,figsize=(12,6.2),sharey=True,sharex=True)
for r,w in enumerate(['3kg','6kg']):
    for c,cond in enumerate(['OFF','ON']):
        sig,ev=S[f'{w}_{cond}']; s0=best_window(ev); seg=sig[s0:s0+8000]; t=np.arange(len(seg))/1000
        a=ax[r,c]; a.axhspan(0,60,color='#f3f2ee',zorder=0); 
        a.axhline(0,color=MUTE,lw=1,ls='--',zorder=1)
        a.plot(t,seg,color=OFFC if cond=='OFF' else ONC,lw=2,zorder=3)
        a.set_ylim(-55,60); a.grid(axis='y'); a.set_xlim(0,8)
        lf=np.median(S[f'{w}_{cond}'][1][:,3]); rf=np.median(S[f'{w}_{cond}'][1][:,2])
        a.set_title(f'{w} 보조 {cond}   (오른발 앞 {rf:.0f}° / 왼발 앞 {lf:.0f}°)',loc='left',color=INK,fontsize=12)
        if c==0: a.set_ylabel('벌어짐 각도 (°)\n= 오른 허벅지 − 왼 허벅지')
        if r==1: a.set_xlabel('시간 (s)')
        if r==0 and c==0:
            a.text(7.9,55,'위쪽: 오른발이 앞',ha='right',va='top',color=INK2,fontsize=10)
            a.text(7.9,-50,'아래쪽: 왼발이 앞',ha='right',va='bottom',color=INK2,fontsize=10)
fig.suptitle(f'그림 1. 두 다리 벌어짐 각도 (참가자 {A.pid}, 왼발목 모래주머니)',x=0.01,ha='left',color=INK,fontsize=14,fontweight='bold')
fig.text(0.01,0.005,'0°(점선) = 두 다리가 나란한 순간. 그래프가 0° 아래로 내려가야 왼발이 오른발보다 앞으로 나간 것. '
         '6kg OFF에서는 거의 0° 아래로 내려가지 못함(왼발이 뒤에 머묾) → 보조 ON에서 위아래 대칭으로 회복.',color=INK2,fontsize=9.5)
fig.tight_layout(rect=(0,0.03,1,0.95)); fig.savefig(OUT+'fig1_split_timeseries.png',dpi=200); plt.close(fig)

# ---------- Fig 2: 전진 각도 막대 ----------
conds=[('0kg_1','0 kg\n(정상)'),('3kg_OFF','3 kg\nOFF'),('3kg_ON','3 kg\nON'),('6kg_OFF','6 kg\nOFF'),('6kg_ON','6 kg\nON')]
rf=[np.median(S[c][1][:,2]) for c,_ in conds]; lf=[np.median(S[c][1][:,3]) for c,_ in conds]
# 0kg: 두 기록 합침
ev0=np.vstack([S['0kg_1'][1],S['0kg_2'][1]]); rf[0]=np.median(ev0[:,2]); lf[0]=np.median(ev0[:,3])
fig,a=plt.subplots(figsize=(10,5.2)); x=np.arange(len(conds)); w=0.36
b1=a.bar(x-w/2-0.01,rf,w,color='#b9b8b3',label='오른발이 앞으로 나간 각도 (건측)')
b2=a.bar(x+w/2+0.01,lf,w,color=ONC,label='왼발이 앞으로 나간 각도 (모래주머니 쪽)')
for i,v in enumerate(lf):
    a.text(x[i]+w/2+0.01,max(v,0)+1.2,f'{v:.0f}°',ha='center',color=INK,fontsize=11,fontweight='bold')
for i,v in enumerate(rf):
    a.text(x[i]-w/2-0.01,v+1.2,f'{v:.0f}°',ha='center',color=INK2,fontsize=10)
a.axhline(0,color=INK2,lw=1)
a.set_xticks(x); a.set_xticklabels([l for _,l in conds]); a.set_ylabel('최대 전진 각도 (°)'); a.grid(axis='y'); a.set_axisbelow(True)
a.set_ylim(-8,52); a.legend(frameon=False,loc='upper left',ncol=2,fontsize=10,bbox_to_anchor=(0,1.02))
a.set_title('그림 2. 걸음마다 각 발이 반대 발보다 앞으로 나간 최대 각도 (걸음별 중앙값)',loc='left',color=INK,fontsize=13,fontweight='bold',pad=28)
fig.text(0.01,0.01,'정상이면 두 막대 높이가 비슷함. 무게가 늘수록 왼발 막대가 낮아지고(6 kg OFF는 −3°: 왼발이 오른발을 한 번도 앞지르지 못함), 보조 ON에서 다시 높아짐.',color=INK2,fontsize=9.5)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(OUT+'fig2_forward_angle.png',dpi=200); plt.close(fig)

# ---------- Fig 3: split_asym 분포 + 통계 ----------
def cliff(a,b): return np.mean(np.sign(np.asarray(a)[:,None]-np.asarray(b)[None,:]))
groups=[('0 kg\n(정상)',ev0[:,1],NORMC),('3 kg\nOFF',S['3kg_OFF'][1][:,1],OFFC),('3 kg\nON',S['3kg_ON'][1][:,1],ONC),
        ('6 kg\nOFF',S['6kg_OFF'][1][:,1],OFFC),('6 kg\nON',S['6kg_ON'][1][:,1],ONC)]
fig,a=plt.subplots(figsize=(10,5.6)); rng=np.random.default_rng(0)
m0,s0=ev0[:,1].mean(),ev0[:,1].std()
a.axhspan(m0-2*s0,m0+2*s0,color='#efeee9',zorder=0); a.text(-0.45,m0-2*s0+0.03,'회색 띠: 정상 범위 (0 kg 평균±2SD)',ha='left',va='bottom',color=INK2,fontsize=9)
a.axhline(0,color=MUTE,lw=1,ls='--')
for i,(lab,v,c) in enumerate(groups):
    a.scatter(i+rng.uniform(-0.17,0.17,len(v)),v,s=14,color=c,alpha=0.55,edgecolors='none',zorder=2)
    q1,md,q3=np.percentile(v,[25,50,75])
    a.plot([i-0.25,i+0.25],[md,md],color=INK,lw=2.5,zorder=4); a.plot([i,i],[q1,q3],color=INK,lw=1.5,zorder=4)
    a.text(i+0.27,md,f'{md:+.2f}',va='center',color=INK,fontsize=10)
for (i,j,key) in [(1,2,'3kg'),(3,4,'6kg')]:
    off,on=groups[i][1],groups[j][1]; p=stats.mannwhitneyu(off,on).pvalue; d=cliff(off,on)
    y=max(np.percentile(off,97),np.percentile(on,97))+0.12
    a.plot([i,i,j,j],[y-0.04,y,y,y-0.04],color=INK2,lw=1)
    a.text((i+j)/2,y+0.02,f'p = {p:.0e}\nCliff δ = {d:.2f}',ha='center',va='bottom',color=INK,fontsize=9.5)
a.set_xticks(range(5)); a.set_xticklabels([g[0] for g in groups]); a.set_xlim(-0.5,4.6)
a.set_ylabel('벌어짐 비대칭 split_asym\n(0 = 대칭, + = 왼발이 덜 나감)'); a.grid(axis='y'); a.set_axisbelow(True)
a.set_ylim(-0.6,1.95)
a.set_title('그림 3. 걸음별 벌어짐 비대칭 (점 하나 = 한 걸음, 굵은 선 = 중앙값, 세로선 = 사분위)',loc='left',color=INK,fontsize=13,fontweight='bold')
fig.text(0.01,0.01,'Mann-Whitney U 검정(걸음 단위, 참가자 1명). Cliff δ: 0 = 차이 없음, ±1 = 두 분포가 완전히 분리. 걸음 시간을 보정한 회귀에서도 3 kg·6 kg 모두 보조 효과 유지(p < 1e-20).',color=INK2,fontsize=9.2)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(OUT+'fig3_split_asym_dist.png',dpi=200); plt.close(fig)

# ---------- Fig 4: 지표 설명표 ----------
rows=[
('벌어짐 각도','오른 허벅지 각도 − 왼 허벅지 각도 (°)','+ 오른발이 앞, − 왼발이 앞. 상체 기울기는 양쪽에 같이 들어가 빼면 사라짐','그림 1'),
('왼발/오른발 전진 각도','한 걸음 안에서 벌어짐 각도의\n최대(오른발)·최소(왼발) 크기 (°)','각 발이 반대 발보다 얼마나 앞으로 나갔나. 정상이면 좌우 비슷','그림 2'),
('벌어짐 비대칭\nsplit_asym','(오른발 전진 − 왼발 전진) ÷ (두 값의 합)','0 = 대칭, + = 왼발이 덜 나감. 1 이상 = 왼발이 오른발을 앞지르지 못함.\n제어가 이 값으로 보조 길이를 조절','그림 3'),
('정상 범위','0 kg 걸음들의 split_asym 평균 ± 2SD','이 사람의 "정상"으로 보는 구간. 보조의 목표','그림 3'),
('보폭 SI','|오른 보폭 − 왼 보폭| ÷ 평균 × 100 (%)\n보폭 = sin(최대굴곡)+sin(최대신전)','각 다리가 흔드는 범위의 좌우 차이. 범위는 같고 앞뒤로 치우친 비대칭은 못 잡음','표'),
('중심 차이','좌우 허벅지 흔들림 중심의 차이 (°)','한쪽 다리가 전체적으로 뒤(또는 앞)에 치우친 정도','표'),
('보행주기 시간 SI','|오른 주기 − 왼 주기| ÷ 평균 × 100 (%)','좌우 걸음 시간의 균형','표'),
('p 값','Mann-Whitney U 검정','OFF와 ON에 차이가 없다고 가정할 때 이만큼 차이가 우연히 나올 확률.\n작을수록(보통 0.05 미만) 차이가 확실','그림 3'),
('Cliff δ','효과크기 (−1 ~ +1)','OFF 걸음이 ON 걸음보다 클 확률 − 작을 확률. |δ|>0.47이면 큰 효과','그림 3'),
]
fig,a=plt.subplots(figsize=(13,6.4)); a.axis('off')
tb=a.table(cellText=[list(r) for r in rows],colLabels=['지표','계산','의미 / 읽는 법','사용'],loc='upper left',
           colWidths=[0.15,0.31,0.47,0.07],cellLoc='left')
tb.auto_set_font_size(False); tb.set_fontsize(10)
for (r,c),cell in tb.get_celld().items():
    cell.set_edgecolor('#e0dfda'); cell.set_linewidth(0.8)
    cell.set_height(0.105 if r>0 else 0.06)
    if r==0: cell.set_facecolor('#efeee9'); cell.get_text().set_fontweight('bold'); cell.get_text().set_color(INK)
    else:
        cell.set_facecolor('#fcfcfb'); cell.get_text().set_color(INK if c!=1 else INK2)
        if c==0: cell.get_text().set_fontweight('bold')
a.set_title('표. 지표 정의',loc='left',color=INK,fontsize=14,fontweight='bold')
fig.tight_layout(); fig.savefig(OUT+'fig4_metric_glossary.png',dpi=200); plt.close(fig)
print('done')
