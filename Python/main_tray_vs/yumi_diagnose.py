"""
Read-only diagnostics for the 'start returns 204 but RAPID stays stopped' problem.

  py yumi_diagnose.py            -> read-only (nothing is changed on the controller)
  py yumi_diagnose.py --start    -> also sends ONE start WITHOUT taking mastership and
                                    polls every 0.1 s for 6 s
                                    (THE ROBOT MAY MOVE - clear the cell first)
  py yumi_diagnose.py --start --master
                                 -> same, but requests rapid+motion mastership first
"""
import json
import sys
import time
import requests
from requests.auth import HTTPDigestAuth

URL = "http://192.168.125.1"
AUTH = HTTPDigestAuth("Zebastian", "robotics")
HEADERS = {
    "Content-Type": "application/x-www-form-urlencoded;v=2.0",
    "Accept": "application/json",
}

s = requests.Session()
s.auth = AUTH
s.headers.update(HEADERS)


def get(path, limit=900):
    sep = "&" if "?" in path else "?"
    try:
        r = s.get(f"{URL}{path}{sep}json=1", timeout=5)
        body = " ".join(r.text.split())
        print(f"\n### GET {path}  ->  {r.status_code}\n{body[:limit]}")
        return r
    except Exception as e:
        print(f"\n### GET {path}  ->  ERROR {e}")
        return None


def states(path):
    try:
        sep = "&" if "?" in path else "?"
        r = s.get(f"{URL}{path}{sep}json=1", timeout=5)
        return json.loads(r.text)["_embedded"]["_state"]
    except Exception:
        return []


def elog(n=10):
    out = []
    for e in states(f"/rw/elog/0?lang=en&order=lifo&limit={n}"):
        out.append(f"  {e.get('tstamp')} [{e.get('code')}] {e.get('title')}")
    return "\n".join(out)


print("=" * 70)
print("CONTROLLER / SYSTEM")
get("/rw/system")                      # RobotWare version: RW6 (IRC5) or RW7?
get("/ctrl")                           # controller info
get("/rw/panel/opmode")
get("/rw/panel/ctrlstate")
get("/rw/panel/speedratio")
get("/rw/mastership")
for dom in ("rapid", "motion", "cfg"):
    get(f"/rw/mastership/{dom}")       # per-domain holder (RW6 uses domains)
get("/rw/rapid/execution")

print("\n" + "=" * 70)
print("RAPID TASKS: program pointer, program and modules per task")
tasks = [t.get("name") for t in states("/rw/rapid/tasks")]
print("Tasks found:", tasks)
for name in tasks:
    print("\n--- ", name)
    get(f"/rw/rapid/tasks/{name}")
    get(f"/rw/rapid/tasks/{name}/pcp")           # program pointer (module/routine)
    get(f"/rw/rapid/modules?task={name}", 1500)   # which modules are loaded (RW6 path)

print("\n" + "=" * 70)
print("EVENT LOG (newest first)")
print(elog(15))

if "--start" in sys.argv:
    print("\n" + "=" * 70)
    print("START TEST (fast polling)")
    use_master = "--master" in sys.argv
    if use_master:
        print("Requesting mastership domains rapid + motion (--master)...")
        print(" mastership (generic):", s.post(f"{URL}/rw/mastership?action=request", timeout=5).status_code)
        for dom in ("rapid", "motion"):
            r0 = s.post(f"{URL}/rw/mastership/{dom}?action=request", timeout=5)
            print(f" mastership {dom}:", r0.status_code, " ".join(r0.text.split())[:150])
        get("/rw/mastership/rapid")
        get("/rw/mastership/motion")
    else:
        print("NO mastership taken (default). Add --master to request rapid+motion domains.")
    get("/rw/panel/ctrlstate")
    print("(Assumes AUTO + motors on + program pointer already at main; "
          "nothing is reset or turned on by this test.)")
    time.sleep(0.5)

    r = s.post(
        f"{URL}/rw/rapid/execution?action=start",
        data={
            "regain": "continue",
            "execmode": "continue",
            "cycle": "forever",
            "condition": "none",
            "stopatbp": "disabled",
            "alltaskbytsp": "false",
        },
        timeout=5,
    )
    print(" start     :", r.status_code, r.text[:300])

    last = None
    t0 = time.time()
    while time.time() - t0 < 6:
        try:
            st = states("/rw/rapid/execution")[0].get("ctrlexecstate")
            ts = ",".join(f"{t.get('name')}={t.get('excstate')}" for t in states("/rw/rapid/tasks"))
            cur = f"{st} | {ts}"
        except Exception as e:
            cur = f"error {e}"
        if cur != last:
            print(f"  t+{time.time() - t0:4.1f}s  {cur}")
            last = cur
        time.sleep(0.1)

    print("\nEvent log after start (newest first):")
    print(elog(12))
    if use_master:
        for dom in ("rapid", "motion"):
            print(f"Releasing mastership {dom}:",
                  s.post(f"{URL}/rw/mastership/{dom}?action=release", timeout=5).status_code)
        print("Releasing mastership (generic):",
              s.post(f"{URL}/rw/mastership?action=release", timeout=5).status_code)

print("\nDone.")
