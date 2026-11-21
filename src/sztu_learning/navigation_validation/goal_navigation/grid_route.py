"""Static-map routing with SciPy Dijkstra/EDT; unknown space is blocked.

Grid stored as OccupancyGrid rows (bottom to top). A circumscribed radius
allows rotation at every route cell. This does not detect new obstacles.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class GridRoute:
    def __init__(self, grid, resolution, origin, radius):
        self.grid = np.asarray(grid)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.radius = float(radius)
        if (self.grid.ndim != 2 or not self.grid.size or self.origin.shape != (3,)
                or not np.isfinite([self.resolution,self.radius,*self.origin]).all()
                or self.resolution<=0 or self.radius<=0):
            raise ValueError('invalid map geometry')
        # Pad blocked borders; include half a pixel diagonal in clearance budget.
        free = self.grid == 0
        distance = distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*self.resolution
        self.allowed = free & (distance > self.radius+self.resolution/math.sqrt(2))
        self.height,self.width = free.shape
        self.graph = self._graph()
        self.manifest_hash = None

    @classmethod
    def load(cls, directory, radius):
        directory=Path(directory)
        manifest=json.loads((directory/'manifest.json').read_text())
        for name in ('occupancy.npy','map.yaml','map.pgm'):
            if sha(directory/name)!=manifest['files_sha256'][name]:
                raise ValueError('map file hash mismatch: '+name)
        cfg=yaml.safe_load((directory/'map.yaml').read_text())
        result=cls(np.load(directory/'occupancy.npy',allow_pickle=False),cfg['resolution'],cfg['origin'],radius)
        result.manifest_hash=sha(directory/'manifest.json')
        return result

    def cell(self, xy):
        dx,dy=np.asarray(xy)-self.origin[:2];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        x=math.floor((c*dx+s*dy)/self.resolution)
        y=math.floor((-s*dx+c*dy)/self.resolution)
        if not (0<=x<self.width and 0<=y<self.height):
            raise ValueError('point outside map')
        return y,x

    def world(self, cell):
        y,x=cell;px,py=(x+.5)*self.resolution,(y+.5)*self.resolution
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return np.array([self.origin[0]+c*px-s*py,self.origin[1]+s*px+c*py])

    def _graph(self):
        ids=np.arange(self.grid.size).reshape(self.grid.shape)
        rows=[];cols=[];weights=[]
        for dy,dx in ((0,1),(1,0),(1,1),(1,-1)):
            ys,xs=np.nonzero(self.allowed)
            ny,nx=ys+dy,xs+dx
            mask=(ny>=0)&(ny<self.height)&(nx>=0)&(nx<self.width)
            ys,xs,ny,nx=(v[mask] for v in (ys,xs,ny,nx))
            mask=self.allowed[ny,nx]
            if dy and dx:mask &= self.allowed[ys,nx]&self.allowed[ny,xs]
            a,b=ids[ys[mask],xs[mask]],ids[ny[mask],nx[mask]]
            rows.extend([a,b]);cols.extend([b,a]);weights.extend([np.full(len(a),math.hypot(dx,dy)*self.resolution)]*2)
        return coo_matrix((np.concatenate(weights),(np.concatenate(rows),np.concatenate(cols))),
                          shape=(self.grid.size,self.grid.size)).tocsr()

    def segment_clear(self,a,b):
        # All cells touched by the segment must be allowed, including corner ties.
        ay,ax=self.cell(a);by,bx=self.cell(b)
        # Grid DDA in local cell coordinates; no sub-cell sampling blind spots.
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        rot=np.array([[c,s],[-s,c]])
        p=rot@(np.asarray(a)-self.origin[:2])/self.resolution
        q=rot@(np.asarray(b)-self.origin[:2])/self.resolution
        delta=q-p;dx=int(np.sign(delta[0]));dy=int(np.sign(delta[1]))
        tx=((ax+(dx>0))-p[0])/delta[0] if dx else math.inf
        ty=((ay+(dy>0))-p[1])/delta[1] if dy else math.inf
        stepx=1/abs(delta[0]) if dx else math.inf
        stepy=1/abs(delta[1]) if dy else math.inf
        def ok(y,x):return 0<=y<self.height and 0<=x<self.width and self.allowed[y,x]
        x,y=ax,ay
        for _ in range(self.width+self.height+2):
            if not ok(y,x):return False
            if (y,x)==(by,bx):return True
            if abs(tx-ty)<1e-10:
                if not ok(y,x+dx) or not ok(y+dy,x):return False
                x+=dx;y+=dy;tx+=stepx;ty+=stepy
            elif tx<ty:x+=dx;tx+=stepx
            else:y+=dy;ty+=stepy
        return False

    def plan(self,start,goal):
        if not np.isfinite([*start,*goal]).all():raise ValueError('nonfinite goal')
        source,target=self.cell(start),self.cell(goal)
        if not self.allowed[source] or not self.allowed[target]:
            raise ValueError('start or goal lacks known clearance')
        if self.segment_clear(start,goal):return [list(start),list(goal)]
        a=np.ravel_multi_index(source,self.grid.shape);b=np.ravel_multi_index(target,self.grid.shape)
        distances,previous=dijkstra(self.graph,directed=False,indices=a,return_predecessors=True)
        if not np.isfinite(distances[b]):raise ValueError('no known free route')
        route=[b]
        while route[-1]!=a:
            route.append(int(previous[route[-1]]))
            if route[-1]<0:raise ValueError('broken route')
        points=[np.asarray(start),*[self.world(np.unravel_index(k,self.grid.shape)) for k in reversed(route)],np.asarray(goal)]
        compact=[points[0]];i=0
        while i<len(points)-1:
            j=len(points)-1
            while j>i+1 and not self.segment_clear(points[i],points[j]):j-=1
            if not self.segment_clear(points[i],points[j]):raise ValueError('route endpoint connection blocked')
            compact.append(points[j]);i=j
        return [point.tolist() for point in compact]
