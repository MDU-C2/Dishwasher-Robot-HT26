"""Independent RGB capture. Default validates only; --live explicitly opens OAK.
No model inference, depth nodes, robot imports or network commands.
"""
import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path


def validate(session, scenes):
    for key in ('session_id', 'operator', 'mount_id', 'lighting', 'camera_settings_note'):
        if not session.get(key) or session[key] == 'TODO':
            raise ValueError('Fill session field: ' + key)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', session['session_id']):
        raise ValueError('session_id: use letters, digits, underscore or dash')
    if session.get('protocol_confirmed') is not True:
        raise ValueError('Confirm protocol before collection')
    if session.get('sensor_resolution') not in ('1080p', '720p'):
        raise ValueError('sensor_resolution must be 1080p or 720p')
    if not scenes:
        raise ValueError('No scenes selected')
    ids = set()
    for scene in scenes:
        sid = scene.get('scene_id', '')
        if not re.fullmatch(r'[A-Za-z0-9_-]+', sid) or sid in ids:
            raise ValueError('Invalid or repeated scene_id: ' + sid)
        ids.add(sid)
        if scene.get('status') != 'planned':
            raise ValueError('Scene must have planned status')
    return session, scenes


def save_frame(root, session, scene, frame, packet_meta, cv):
    """Immutable image and sidecar; sidecar is the authoritative capture manifest row."""
    image_path = root / 'images' / (scene['scene_id'] + '.png')
    meta_path = root / 'metadata' / (scene['scene_id'] + '.json')
    if image_path.exists() or meta_path.exists():
        raise FileExistsError('Existing capture or partial write: ' + scene['scene_id'])
    ok, encoded = cv.imencode('.png', frame)
    if not ok:
        raise OSError('PNG encoding failed; not counted')
    payload = encoded.tobytes()
    record = dict(session_id=session['session_id'], scene_id=scene['scene_id'],
                  placement_id=session['session_id'] + '_' + scene['scene_id'],
                  captured_at_utc=datetime.now(timezone.utc).isoformat(),
                  image_path='images/' + image_path.name, sha256=hashlib.sha256(payload).hexdigest(),
                  width=int(frame.shape[1]), height=int(frame.shape[0]),
                  source='OAK RGB video; BGR from getCvFrame encoded as PNG',
                  plan=scene, session=session, device=packet_meta,
                  independent_reposition_operator_confirmed=True,
                  annotation_status='pending', review_status='pending',
                  measured_distance_cm=None, measured_x_cm=None, cup_ids=[],
                  object_count=None, exclusions=[], split='candidate_final_test')
    with image_path.open('xb') as f:
        f.write(payload)
    with meta_path.open('x', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--live', action='store_true')
    a = p.parse_args()
    session = json.loads(a.session.read_text(encoding='utf-8-sig'))
    scenes = [json.loads(x) for x in a.plan.read_text(encoding='utf-8-sig').splitlines() if x.strip()]
    validate(session, scenes)
    if not a.live:
        print(f'Plan valid: {len(scenes)} planned rows. Camera NOT opened.')
        return
    import cv2 as cv
    import depthai as dai
    if dai.__version__ != '2.30.0.0':
        raise RuntimeError('Use existing DepthAI 2.30.0.0 environment')
    root = a.output.resolve() / session['session_id']
    root.mkdir(parents=True, exist_ok=True)
    for folder in ('images', 'metadata', 'labels'):
        (root / folder).mkdir(exist_ok=True)
    # Never silently resume or overwrite a prior session.
    with (root / 'session.json').open('x', encoding='utf-8') as f:
        json.dump(session, f, ensure_ascii=False, indent=2)
    (root / 'plan.jsonl').write_text('\n'.join(json.dumps(s, ensure_ascii=False) for s in scenes)+'\n', encoding='utf-8')
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    resolution = dai.ColorCameraProperties.SensorResolution
    cam.setResolution(resolution.THE_1080_P if session['sensor_resolution'] == '1080p' else resolution.THE_720_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(30)
    out = pipeline.create(dai.node.XLinkOut)
    out.setStreamName('rgb')
    cam.video.link(out.input)
    index, armed, armed_at = 0, False, 0
    import time
    try:
        with dai.Device(pipeline) as device:
            queue = device.getOutputQueue('rgb', maxSize=1, blocking=False)
            while index < len(scenes):
                packet = queue.tryGet()
                if packet is None:
                    if cv.waitKey(10) & 0xff == ord('q'):
                        break
                    continue
                frame = packet.getCvFrame()
                scene = scenes[index]
                display = frame.copy()
                cv.putText(display, f"{scene['scene_id']}  {scene.get('orientation', '')}  {scene['scenario']}", (20, 35), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
                cv.putText(display, 'R=confirm NEW placement | SPACE=save | Q=quit', (20, 65), cv.FONT_HERSHEY_SIMPLEX, .6, (0,255,255), 2)
                cv.putText(display, 'ARMED' if armed else 'Reposition, then press R', (20, 95), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
                cv.putText(display, f"Band: {scene.get('distance_band')} | X target cm: {scene.get('target_x_cm', 'see plan')}", (20, 125), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
                cv.imshow('Independent RGB capture', display)
                key = cv.waitKey(1) & 0xff
                if key == ord('q'):
                    break
                if key == ord('r') and not armed:
                    armed, armed_at = True, time.monotonic()
                if key == ord(' ') and armed and time.monotonic() - armed_at >= 2:
                    info = dict(mxid=device.getMxId(), depthai=dai.__version__, sequence=packet.getSequenceNum(),
                                device_timestamp_s=packet.getTimestampDevice().total_seconds(),
                                exposure_us=packet.getExposureTime().total_seconds()*1e6,
                                iso=packet.getSensitivity(), lens_position=packet.getLensPosition(),
                                color_temperature=packet.getColorTemperature())
                    record = save_frame(root, session, scene, frame, info, cv)
                    print('Saved; annotation pending:', record['image_path'])
                    index += 1
                    armed = False
    finally:
        cv.destroyAllWindows()
    print(f'Saved this session: {index}. These are candidates, not accepted test images.')


if __name__ == '__main__':
    main()
