import json,glob,collections,numpy as np
def failed(r,sc):
    if r['diverged']: return True
    if sc=='S3': return r.get('rec_1') is None
    return r['rmse']>3.0
