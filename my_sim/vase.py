#!/usr/bin/env python3
import time, threading, argparse, socket, json, queue, sys, signal
import gymnasium, manipulator_mujoco, mujoco
from transform_utils import quaternion_to_rotation_matrix

# ------------------- метрики -------------------
M = {"Всего": 0, "Захвачено": 0, "Поднято": 0, "Удержено": 0}
def pct(k): return 100.0*M[k]/M["Всего"] if M["Всего"] else 0.0
def summary(fin=False):
    if not M["Всего"]: return
    tag = "ИТОГО" if fin else "СТАТ"
    print(f"\n[{tag}] N={M['Всего']} | Захвачено={pct('Захвачено'):.1f}% | "
          f"Поднято={pct('Поднято'):.1f}% | Удержено={pct('Удержено'):.1f}%\n")

# ------------------- пороги --------------------
OPEN, CLOSED            = 0.0, 0.943
CLOSE_FORCE_LIMIT       = 600.0      # защитный остановка при закрытии
CAPTURE_THRESHOLD       = 100.0       # ≥ — «захвачен»
HOLD_THRESHOLD          = 80.0        # ≥ — «удержан» в конце
LIFT_VALIDATION_WINDOW  = 1.2         # сек
MIN_SAMPLES_ABOVE       = 60           # сколько выборок ≥ CAPTURE_THRESHOLD нужно

def sim_loop(ip, port):
    env  = gymnasium.make("manipulator_mujoco/AuboI5EnvVase-v0", render_mode="human")
    phys = env.unwrapped._physics
    q    = queue.Queue()

    # ---------- socket ----------
    def sock():
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((ip, port)); s.listen(1)
            print(f"[Socket] ждём {ip}:{port}…")
            c, _ = s.accept(); print("[Socket] подключено.")
            buf=""
            while True:
                ch=c.recv(4096)
                if not ch: break
                buf+=ch.decode()
                while "\n" in buf:
                    ln, buf = buf.split("\n",1)
                    try: q.put(json.loads(ln.strip()))
                    except json.JSONDecodeError: print("[Socket] bad JSON", ln.strip())
    threading.Thread(target=sock, daemon=True).start()

    # ---------- ids ----------
    gid = mujoco.mj_name2id(phys.model.ptr, mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
    lid = mujoco.mj_name2id(phys.model.ptr, mujoco.mjtObj.mjOBJ_SENSOR,   "left_finger_force")
    rid = mujoco.mj_name2id(phys.model.ptr, mujoco.mjtObj.mjOBJ_SENSOR,   "right_finger_force")

    # ---------- FSM ----------
    def run(tr):
        env.reset(seed=42)
        state="random"
        pre_t=fin_t=lift_t=None
        grip=OPEN
        closing=False; close_t=None
        captured=False; lift_confirmed=False; held=False
        last_log=0

        # — для проверки устойчивого подъёма —
        lift_samples_tot = 0
        lift_samples_above = 0

        while True:
            force = phys.data.sensordata[lid]+phys.data.sensordata[rid]

            # ---- управление схватом ----
            if closing:
                a=min(1.0,(time.time()-close_t)/3.0)
                grip=OPEN+a*(CLOSED-OPEN)
                if force>=CLOSE_FORCE_LIMIT or a>=1.0:
                    closing=False
            phys.data.ctrl[gid]=grip

            # ---- лог каждые 0.5 с ----
            if time.time()-last_log>=.2:
                print(f"[{state:^11}] F={force:7.1f} N  grip={grip:.3f}")
                last_log=time.time()

            # ---- переходы ----
            if state=="random":
                env.step(env.action_space.sample())
                if tr:
                    safe=[tr["translation"][0],tr["translation"][1],tr["translation"][2]+.3]
                    env.unwrapped._target.set_mocap_pose(phys, position=safe, quaternion=tr["rotation"])
                    pre_t=time.time(); state="pre_grasp"; print("[-> pre_grasp]")

            elif state=="pre_grasp":
                env.step([0]*env.action_space.shape[0])
                if time.time()-pre_t>=10:
                    env.unwrapped._target.set_mocap_pose(phys, position=tr["translation"], quaternion=tr["rotation"])
                    fin_t=time.time(); state="final_grasp"; print("[-> final_grasp]")

            elif state=="final_grasp":
                env.step(env.action_space.sample())
                if not closing and time.time()-fin_t>=3.0 and grip<CLOSED*.95:
                    closing=True; close_t=time.time(); print("[Gripper] closing")
                # --- захват ---
                if not captured and grip>=CLOSED*.95 and force>=CAPTURE_THRESHOLD:
                    captured=True; print(f"[Metric] Захвачено ✓  F={force:.1f}")
                # после 5 с пробуем подъём
                if grip>=CLOSED*.95 and time.time()-fin_t>=5.0:
                    lift_t=time.time(); state="lift"; print("[-> Поднято]")

            elif state=="lift":
                env.step(env.action_space.sample())
                # задаём цель только вначале
                if time.time()-lift_t<.1:
                    p=tr["translation"]; env.unwrapped._target.set_mocap_pose(
                        phys, position=[p[0],p[1],p[2]+.4], quaternion=tr["rotation"])
                # собираем статистику силы
                if time.time()-lift_t <= LIFT_VALIDATION_WINDOW:
                    lift_samples_tot   += 1
                    if force >= CAPTURE_THRESHOLD:
                        lift_samples_above += 1
                elif not lift_confirmed:
                    # переход завершился: решаем, засчитан ли подъём
                    lift_confirmed = lift_samples_above >= MIN_SAMPLES_ABOVE
                    print(f"[Metric] Поднято {'✓' if lift_confirmed else '✗'}  "
                          f"above={lift_samples_above}/{lift_samples_tot}")
                # по таймеру -> done
                if time.time()-lift_t>5.0:
                    state="done"; print("[-> done]")

            elif state=="done":
                env.step([0]*env.action_space.shape[0])
                held = captured and lift_confirmed and force>=HOLD_THRESHOLD
                print(f"[DONE] F_end={force:.1f} N  held={held}")
                return captured, lift_confirmed, held

    # ---------- Ctrl‑C ----------
    signal.signal(signal.SIGINT, lambda *_: (summary(True), env.close(), sys.exit(0)))

    print("[Main] ждём пересчета.")
    while True:
        tr=q.get()
        if tr is None: break
        c,l,h=run(tr)
        M["Всего"]+=1
        M["Захвачено"]+=c
        M["Поднято"]   += (c and l)
        M["Удержено"]   += h
        summary()

    summary(True); env.close()

if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--ip",default="127.0.0.1"); p.add_argument("--port",type=int,default=54321)
    a=p.parse_args(); sim_loop(a.ip,a.port)
