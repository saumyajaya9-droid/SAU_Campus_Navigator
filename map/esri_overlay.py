import numpy as np,math,sys,cv2
sys.path.insert(0,"/home/claude")
import importlib, campus_geo as G
k=111320*math.cos(math.radians(28.485)); m=lambda ll: np.array([ll[1]*k,ll[0]*110540])
pts={"Gate3":(63,192),"ConventionCentre":(304,432),"Amphitheater":(233,680),"J1":(153,237)}
X=np.array([m(G.GEO[n]) for n in pts]); P=np.array(list(pts.values()),float); A=np.c_[X,np.ones(len(X))]
cx=np.linalg.lstsq(A,P[:,0],rcond=None)[0]; cy=np.linalg.lstsq(A,P[:,1],rcond=None)[0]
to=lambda ll: (int(np.r_[m(ll),1]@cx), int(np.r_[m(ll),1]@cy))
im=cv2.imread("/mnt/user-data/attachments/87d7c525-6fa4-5412-b0ec-77f6fe7d9c58/238bc806-image.png")
import json
SH=json.load(open(sys.argv[2])) if len(sys.argv)>2 else G.ROAD_SHAPES
for key,sh in SH.items():
    a,b=key.split("|"); col=(0,0,255) if (a[0]=="J" or a[:4]=="Gate") and (b[0]=="J" or b[:4]=="Gate") else (255,0,0)
    cv2.polylines(im,[np.array([to(p) for p in sh],np.int32)],False,col,2)
for n,p in G.GEO.items(): cv2.circle(im,to(p),4,(0,255,0),-1)
cv2.imwrite(sys.argv[1],cv2.resize(im,None,fx=1.6,fy=1.6))
