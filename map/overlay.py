import json,cv2,numpy as np,math,sys
sys.path.insert(0,"/home/claude")
from campus_geo import GEO
S="/tmp/claude-0/-home-claude/87d7c525-6fa4-5412-b0ec-77f6fe7d9c58/scratchpad/"
R=json.load(open(sys.argv[1]))
im=cv2.imread(S+"img/sat.png")
k=111320*math.cos(math.radians(28.485)); af=np.load("/home/claude/affine.npy"); M=af[:4].reshape(2,2); t=af[4:]
def px(ll):
    v=M@np.array([ll[1]*k,ll[0]*110540])+t; return (int(v[0]),int(v[1]))
for key,v in R["links"].items():
    cv2.polylines(im,[np.array([px(p) for p in v["shape"]],np.int32)],False,(255,255,0),2)
for key,v in R["roads"].items():
    cv2.polylines(im,[np.array([px(p) for p in v["shape"]],np.int32)],False,(0,128,255),3)
for n,p in GEO.items(): cv2.circle(im,px(p),4,(0,0,255),-1)
cv2.imwrite(sys.argv[2],im[0:1285,0:1065])
