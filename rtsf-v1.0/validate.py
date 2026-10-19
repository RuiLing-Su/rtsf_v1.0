#!/usr/bin/env python3
"""Validate RTSF messages and examples without network access."""
import argparse
import json
import math
from pathlib import Path

from jsonschema import Draft202012Validator
from referencing import Registry, Resource

ROOT = Path(__file__).resolve().parent
MAX_BYTES = 65536
SCHEMAS = [json.loads(p.read_text()) for p in sorted((ROOT / 'schemas').glob('*.json'))]
REGISTRY = Registry().with_resources((s['$id'], Resource.from_contents(s)) for s in SCHEMAS)
VALIDATOR = Draft202012Validator(next(s for s in SCHEMAS if s['$id'].endswith('/rtsf.json')), registry=REGISTRY)
PROFILE = Draft202012Validator(next(s for s in SCHEMAS if s['$id'].endswith('/path_capabilities.json')), registry=REGISTRY)
ROUTES = {'device.state': 'properties/device', 'map.info': 'properties/map',
          'task.state': 'properties/task', 'task.state_changed': 'events',
          'device.fault': 'events', 'trajectory.batch': 'trajectory'}
SERVICES = ('task.path.set', 'task.start', 'task.pause', 'task.resume', 'task.cancel')
TRANSITIONS = {None: ('ACCEPTED',), 'ACCEPTED': ('RUNNING', 'CANCELED', 'FAILED'),
               'RUNNING': ('PAUSED', 'COMPLETED', 'CANCELED', 'FAILED'),
               'PAUSED': ('RUNNING', 'CANCELED', 'FAILED')}


def finite(value):
    if isinstance(value, dict):
        return all(finite(v) for v in value.values())
    if isinstance(value, list):
        return all(finite(v) for v in value)
    if isinstance(value, (int, float)):
        try:
            return math.isfinite(value)
        except OverflowError:
            return False
    return True


def loads(raw):
    if len(raw) > MAX_BYTES:
        raise ValueError('JSON exceeds 65536 UTF-8 bytes')

    def constant(value):
        raise ValueError('Invalid numeric constant: ' + value)

    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise ValueError('Duplicate key: ' + key)
            out[key] = value
        return out

    value = json.loads(raw.decode('utf-8'), parse_constant=constant, object_pairs_hook=pairs)
    if not finite(value):
        raise ValueError('Non-finite or unrepresentable number')
    return value


def topic_for(msg):
    method = msg['method']
    suffix = 'services_reply' if 'request_id' in msg else 'services' if method in SERVICES else ROUTES[method]
    return 'rtsf/v1/' + msg['robot_id'] + '/' + suffix


def check(raw, profile=False, topic=None):
    try:
        msg = loads(raw)
    except (ValueError, UnicodeError, OverflowError, RecursionError) as error:
        return [str(error)]
    try:
        errors = list(VALIDATOR.iter_errors(msg))
    except RecursionError:
        return ['JSON nesting exceeds validator capacity']
    if errors:
        return ['Schema: ' + errors[0].message[:180]]
    data, method = msg['data'], msg['method']
    errors = []
    if method == 'task.path.set' and 'request_id' not in msg:
        if [p['seq'] for p in data['path']['waypoints']] != list(range(len(data['path']['waypoints']))):
            errors.append('Waypoint seq must start at zero and be continuous')
        if profile:
            errors.extend('Capability profile: ' + e.message[:120] for e in PROFILE.iter_errors(msg))
    if method == 'trajectory.batch':
        times = [s['t'] for s in data['samples']]
        if any(b <= a for a, b in zip(times, times[1:])):
            errors.append('Sample t must be strictly increasing')
    if method == 'map.info':
        b = data['bounds']
        if b['max_x'] <= b['min_x'] or b['max_y'] <= b['min_y']:
            errors.append('Bounds max must exceed min')
    if method == 'task.state_changed' and data['status'] not in TRANSITIONS[data['previous_status']]:
        errors.append('Invalid task state transition')
    if topic is not None and topic != topic_for(msg):
        errors.append('Topic does not match method, robot_id or response role')
    return errors


def selftest():
    import hashlib

    checks = failures = 0

    def expect(label, errors, accepted):
        nonlocal checks, failures
        checks += 1
        passed = (not errors) == accepted
        failures += not passed
        if not passed:
            print('FAIL', label, errors)

    def sample(name):
        return json.loads((ROOT / 'examples' / (name + '.json')).read_text())

    def test(label, msg, accepted, profile=False):
        raw = json.dumps(msg, allow_nan=False, separators=(',', ':')).encode()
        expect(label, check(raw, profile=profile), accepted)

    for schema in SCHEMAS:
        Draft202012Validator.check_schema(schema)
    examples = sorted((ROOT / 'examples').glob('*.json'))
    for path in examples:
        raw = path.read_bytes()
        msg = loads(raw)
        topic = topic_for(msg)
        expect(path.name, check(raw, profile=True, topic=topic), True)
        expect(path.name + ': wrong device', check(raw, topic=topic.replace(msg['robot_id'], 'other-device')), False)

    cases = [
        ('task_start', 'missing data', lambda m: m.pop('data')),
        ('task_start', 'missing task', lambda m: m['data'].pop('task_id')),
        ('task_start', 'missing message ID', lambda m: m.pop('message_id')),
        ('task_start', 'missing version', lambda m: m.pop('schema_version')),
        ('task_start', 'negative timestamp', lambda m: m.update(timestamp=-1)),
        ('task_start', 'timestamp overflow', lambda m: m.update(timestamp=2**63)),
        ('service_accepted', 'missing request ID', lambda m: m.pop('request_id')),
        ('service_accepted', 'accepted without task', lambda m: m['data'].pop('task_id')),
        ('service_accepted', 'accepted with error', lambda m: m['data'].update(error_code='INTERNAL_ERROR')),
        ('service_rejected', 'rejected without error', lambda m: m['data'].pop('error_code')),
        ('service_accepted', 'accepted unknown method', lambda m: m.update(method='task.unknown')),
        ('device_state', 'reply ID on property', lambda m: m.update(request_id=m['message_id'])),
        ('device_offline', 'invalid offline source', lambda m: m['data'].update(source='REPORT')),
        ('task_state', 'progress overflow', lambda m: m['data'].update(progress=1.1)),
        ('task_state', 'revision zero', lambda m: m['data'].update(revision=0)),
        ('task_state', 'failed without reason', lambda m: m['data'].update(status='FAILED')),
        ('task_state_changed', 'no transition', lambda m: m['data'].update(previous_status=m['data']['status'])),
        ('map_info', 'reversed bounds', lambda m: m['data']['bounds'].update(min_x=999)),
        ('trajectory_batch', 'missing reference', lambda m: m['data'].pop('base_frame_id')),
        ('trajectory_batch', 'missing final flag', lambda m: m['data'].pop('final')),
        ('trajectory_batch', 'empty ordinary batch', lambda m: m['data'].update(samples=[])),
        ('trajectory_batch', 'negative speed', lambda m: m['data']['samples'][0].update(speed_mps=-1)),
        ('trajectory_batch', 'duplicate sample time', lambda m: m['data']['samples'][1].update(t=m['data']['samples'][0]['t'])),
        ('task_path_set', 'missing expiry', lambda m: m['data'].pop('expire_at')),
        ('task_path_set', 'empty waypoints', lambda m: m['data']['path'].update(waypoints=[])),
        ('task_path_set', 'sequence gap', lambda m: m['data']['path']['waypoints'][1].update(seq=3)),
        ('task_path_set', 'sequence origin', lambda m: m['data']['path']['waypoints'][0].update(seq=1)),
        ('task_path_set', 'string coordinate', lambda m: m['data']['path']['waypoints'][0].update(x='1')),
    ]
    for name, label, mutate in cases:
        msg = sample(name)
        mutate(msg)
        test(label, msg, False)
    for field, value in [('type', 'VIA'), ('speed_mps', .2), ('tolerance_m', .1), ('action', 'PHOTO'), ('action_params', {})]:
        msg = sample('task_path_set')
        msg['data']['path']['waypoints'][0][field] = value
        test('reserved ' + field, msg, True)
        test('unsupported ' + field, msg, False, profile=True)
    msg = sample('task_path_set')
    point = msg['data']['path']['waypoints'][0]
    msg['data']['path']['waypoints'] = [dict(point, seq=i) for i in range(500)]
    test('500 waypoints', msg, True, profile=True)
    msg['data']['path']['waypoints'].append(dict(point, seq=500))
    test('501 waypoints', msg, False)
    for name, mutate in [
        ('device_fault', lambda m: m['data'].update(active=False)),
        ('device_offline', lambda m: m['data'].update(source='SHUTDOWN')),
        ('task_start', lambda m: m.update(extension='ignored')),
        ('service_rejected', lambda m: (m.update(method='task.unknown'), m['data'].update(error_code='METHOD_UNSUPPORTED'))),
    ]:
        msg = sample(name)
        mutate(msg)
        test(name + ': variant', msg, True)
    msg = sample('task_start')
    msg['padding'] = ''
    raw = json.dumps(msg, separators=(',', ':')).encode()
    msg['padding'] = ' ' * (MAX_BYTES - len(raw))
    raw = json.dumps(msg, separators=(',', ':')).encode()
    assert len(raw) == MAX_BYTES
    expect('65536 bytes', check(raw), True)
    expect('65537 bytes', check(raw + b' '), False)
    for number in ('NaN', 'Infinity', '-Infinity', '1e999'):
        raw = json.dumps(sample('task_start'))[:-1] + ', "extension": ' + number + '}'
        expect(number, check(raw.encode()), False)
    raw = json.dumps(sample('task_start'))[:-1] + ', "extra": 1, "extra": 2}'
    expect('duplicate key', check(raw.encode()), False)
    info = sample('map_info')['data']
    digest = hashlib.sha256((ROOT / 'examples/map/map-001.zip').read_bytes()).hexdigest()
    expect('map ZIP', [] if digest == info['map_file']['sha256'] else ['SHA256 mismatch'], True)
    print(f'{checks} checks, {failures} failures; {len(SCHEMAS)} schemas checked')
    return bool(failures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--topic', help='Also validate exact MQTT topic routing')
    parser.add_argument('files', nargs='*')
    args = parser.parse_args()
    if not args.files:
        if args.topic or args.profile:
            parser.error('--topic and --profile require a message file')
        return selftest()
    failed = False
    for filename in args.files:
        try:
            errors = check(Path(filename).read_bytes(), args.profile, args.topic)
        except OSError as error:
            errors = [str(error)]
        failed |= bool(errors)
        print(('FAIL' if errors else 'PASS'), filename, '; '.join(errors))
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
