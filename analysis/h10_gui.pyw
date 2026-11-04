# -*- coding: utf-8 -*-
"""H10 로그 분석 창.  이 파일을 더블클릭하면 열립니다 (h10_analyze.py 와 같은 폴더에 두세요)."""
import os, sys, traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from h10_analyze import analyze  # noqa: E402

CONDS = ['check', '0kg_OFF_1', '0kg_OFF_2',
         '3kg_OFF', '3kg_TUNE1', '3kg_TUNE2', '3kg_TUNE3', '3kg_ON',
         '6kg_OFF', '6kg_TUNE1', '6kg_TUNE2', '6kg_TUNE3', '6kg_ON']
HELP = {
    'check': '착용 확인 (20초 정도면 됨)',
    '0kg_OFF_1': '정상값 1차: 0 kg, 보조 OFF',
    '0kg_OFF_2': '정상값 2차: 0 kg, 보조 OFF (재현성)',
    '3kg_OFF': '왼발목 3 kg, 보조 OFF',
    '3kg_TUNE1': '3 kg 보조량 조정 1 (짧게 5~10주기)',
    '3kg_TUNE2': '3 kg 보조량 조정 2', '3kg_TUNE3': '3 kg 보조량 조정 3',
    '3kg_ON': '3 kg 보조 ON 본 측정 (보조량 고정)',
    '6kg_OFF': '왼발목 6 kg, 보조 OFF',
    '6kg_TUNE1': '6 kg 보조량 조정 1 (3 kg 확정값부터 시작)',
    '6kg_TUNE2': '6 kg 보조량 조정 2', '6kg_TUNE3': '6 kg 보조량 조정 3',
    '6kg_ON': '6 kg 보조 ON 본 측정 (보조량 고정)',
}

root = tk.Tk(); root.title('H10 보행 분석'); root.geometry('860x640')
font = ('맑은 고딕', 10); mono = ('Consolas', 10)
frm = ttk.Frame(root, padding=10); frm.pack(fill='x')

v_log = tk.StringVar(); v_pid = tk.StringVar(value='A'); v_cond = tk.StringVar(value='0kg_OFF_1')
v_gain = tk.StringVar(value='0.0'); v_side = tk.StringVar(value='LH')

ttk.Label(frm, text='로그 파일', font=font).grid(row=0, column=0, sticky='w')
ttk.Entry(frm, textvariable=v_log, width=70).grid(row=0, column=1, columnspan=3, sticky='we', padx=4)
ttk.Button(frm, text='찾기...', command=lambda: v_log.set(filedialog.askopenfilename(
    filetypes=[('H10 로그', '*.csv'), ('모든 파일', '*.*')]) or v_log.get())).grid(row=0, column=4)

ttk.Label(frm, text='참가자', font=font).grid(row=1, column=0, sticky='w', pady=6)
ttk.Entry(frm, textvariable=v_pid, width=8).grid(row=1, column=1, sticky='w', padx=4)
ttk.Label(frm, text='조건', font=font).grid(row=1, column=2, sticky='e')
cb = ttk.Combobox(frm, textvariable=v_cond, values=CONDS, state='readonly', width=14); cb.grid(row=1, column=3, sticky='w', padx=4)
lbl_help = ttk.Label(frm, text=HELP['0kg_OFF_1'], font=font, foreground='#555'); lbl_help.grid(row=2, column=1, columnspan=4, sticky='w')

ttk.Label(frm, text='넣어 둔 보조량 [Nm]', font=font).grid(row=3, column=0, sticky='w', pady=6)
e_gain = ttk.Entry(frm, textvariable=v_gain, width=8); e_gain.grid(row=3, column=1, sticky='w', padx=4)
ttk.Label(frm, text='모래주머니 쪽', font=font).grid(row=3, column=2, sticky='e')
sf = ttk.Frame(frm); sf.grid(row=3, column=3, sticky='w')
ttk.Radiobutton(sf, text='왼다리', value='LH', variable=v_side).pack(side='left')
ttk.Radiobutton(sf, text='오른다리', value='RH', variable=v_side).pack(side='left')

def on_cond(*_):
    c = v_cond.get(); lbl_help.config(text=HELP.get(c, ''))
    off = c.startswith('0kg') or c == 'check' or c.endswith('_OFF')
    e_gain.config(state='disabled' if off else 'normal')
    if off: v_gain.set('0.0')
cb.bind('<<ComboboxSelected>>', on_cond)

txt = tk.Text(root, font=mono, wrap='word', height=24); txt.pack(fill='both', expand=True, padx=10, pady=(0, 10))
for tag, kw in {'h': dict(font=('맑은 고딕', 11, 'bold')), 'ok': dict(foreground='#1a7f37'),
                'bad': dict(foreground='#c62828', font=('맑은 고딕', 10, 'bold')),
                'act': dict(foreground='#0b57d0', font=('맑은 고딕', 11, 'bold')), 'n': {}}.items():
    txt.tag_config(tag, **kw)

def run():
    log = v_log.get().strip().strip('"')
    if not os.path.isfile(log):
        messagebox.showwarning('파일 없음', '로그 파일을 선택하세요.'); return
    try:
        gain = float(v_gain.get() or 0)
    except ValueError:
        messagebox.showwarning('보조량', '보조량은 숫자로 입력하세요.'); return
    c = v_cond.get()
    short = c == 'check' or 'TUNE' in c
    txt.delete('1.0', 'end'); txt.insert('end', '분석 중...\n'); root.update()
    try:
        lines, _ = analyze(log, v_pid.get().strip() or 'A', c, v_side.get(), gain,
                           outdir=os.path.join(HERE, 'results'), min_pairs=5 if short else 10)
        txt.delete('1.0', 'end')
        for s, k in lines:
            txt.insert('end', s + '\n', k)
    except Exception as e:
        txt.delete('1.0', 'end'); txt.insert('end', '오류: ' + str(e) + '\n\n' + traceback.format_exc(), 'bad')

bf = ttk.Frame(frm); bf.grid(row=4, column=0, columnspan=5, sticky='w', pady=6)
ttk.Button(bf, text='분석 실행', command=run).pack(side='left')
ttk.Button(bf, text='결과 폴더 열기', command=lambda: (os.makedirs(os.path.join(HERE, 'results'), exist_ok=True),
           os.startfile(os.path.join(HERE, 'results')))).pack(side='left', padx=8)
frm.columnconfigure(1, weight=1)
root.mainloop()
