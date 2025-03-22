#%%
import yaml
from yaml import SafeDumper
import numpy as np
def represent_flow_style_list(dumper, data):
    return dumper.represent_sequence(yaml.resolver.BaseResolver.DEFAULT_SEQUENCE_TAG, data, flow_style=True)
SafeDumper.add_representer(list, represent_flow_style_list)
#%%
class FlowListDumper(yaml.Dumper):
    def represent_list(self, data):
        return self.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

def load_ex(path,mode,ref='rear_axle_center'):
    with open(path) as stream:
        y = yaml.safe_load(stream)
    assert y['reference'] == ref
    if mode == 'matrix':
        ret = np.zeros((3,4))
        ret[0:3,0:3] = y['rotation']
        ret[:,3] = y['position']
        return ret
    elif mode == 'tuple':
        return np.array(y['rotation']),np.array(y['position'])


def save_ex(path,rotation=None,translation=None,matrix=None,ref='rear_axle_center'):
    if matrix is not None:
        rot = matrix[0:3,0:3]
        trans = matrix[0:3,3]
        save_ex(path,rot,trans,ref=ref)
        return
    ret = {}
    ret['reference'] = ref
    ret['rotation'] = rotation
    ret['translation'] = translation
    for i in ret:
        if type(ret[i]) == np.ndarray:
            ret[i] = ret[i].tolist()
    print(yaml.dump(ret,Dumper=SafeDumper,default_flow_style=False))
    with open(path,'w') as stream:
        yaml.dump(ret,stream,Dumper=SafeDumper,default_flow_style=False)

def load_in(path,mode='matrix'):
    with open(path) as stream:
        y = yaml.safe_load(stream)
    if mode == 'matrix':
        ret = np.diag(y['focal'])
        ret[0:2,2] = y['center']
        return ret
    elif mode == 'tuple':
        return np.array(y['focal']),np.array(y['center'])

def save_in(path,focal=None,center=None,matrix=None):
    if matrix is not None:
        focal = matrix.diagonal()
        center = matrix[0:2,2]
        save_in(path,focal,center)
        return
    ret = {}
    ret['focal'] = focal
    ret['center'] = center
    for i in ret:
        if type(ret[i]) == np.ndarray:
            ret[i] = ret[i].tolist()
    print(yaml.dump(ret,Dumper=SafeDumper,default_flow_style=False))
    with open(path,'w') as stream:
        yaml.dump(ret,stream,Dumper=SafeDumper,default_flow_style=False)


#%%
if __name__ == "__main__":
    #%%
    rot, trans = load_ex('/mnt/GEMstack/GEMstack/knowledge/calibration/gem_e4_ouster.yaml',mode='tuple')
    save_ex('/tmp/test.yaml',rot,trans)
    #%%
    focal = [1,2,3]
    center = [400,500]
    save_in('/tmp/test.yaml',focal,center)
    load_in('/tmp/test.yaml',mode='tuple')
