#!/usr/bin/env python


def build_labels(order):
    lookup = {'b': 'blend', 's': 'shared', 'a': 'auto', 't': 'tele'}
    return [a for c in order for a in [lookup[c]]*5 ]

mapping = {
    'p10': {
        'gaze': ['%03d' % i for i in range(19)],
        'video': ['%s%d' % (s,d) for s,d in zip(build_labels('bs') , range(1,6)*2)] +
            ['%s%d' % (s,d) for s,d in zip(build_labels('t') , range(1,6))] +
            ['%s%d' % (s,d) for s,d in zip(build_labels('a') , range(1,6))]
        }
    }


def get_index(video):
    p1 = video.find('_')
    p2 = video.find('.')
    if p2 >= 0:
        video = video[:p2]
    
    lst = mapping[video[:p1]]
    idx = lst['video'].index(video[p1+1:])
    return lst['gaze'][idx]
