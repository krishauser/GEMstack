# Create Grid Map
import heapq
import math

self.grid_map = np.full((self.s_bound[0], self.s_bound[1]), np.inf)

def obstacle_heuristic():
    print("self.end",self.end)
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1),(-1,-1),(1,1),(1,-1),(-1,1)]
    if len(self.grid_map)==0:
        return
    end = self.end
    n,m = len(self.grid_map),len(self.grid_map[0])
    pq = [(0,end[0],end[1])]
    print("nnnnn",n)
    print("mmmmm",m)
    end_s = self.state2s(end)
    self.grid_map[end_s[0]][end_s[1]] = 0
    cnt = 0
    delta = 1
    while pq:
        print("heapq is",pq)
        cost,x,y = heapq.heappop(pq)
        cnt += 1
        s_xy = self.state2s([x,y,self.end[2],self.end[3]])
        if cost>self.grid_map[s_xy[0]][s_xy[1]]:
            continue

        for idx,(dx,dy) in enumerate(directions):
            nx,ny = x + dx*delta,y + dy*delta
            s_nxny = self.state2s([nx,ny,self.end[2],self.end[3]])
            if  0 <= s_nxny[0] < n and 0 <= s_nxny[1] < m and check_constraints([nx,ny,self.end[2],self.end[3]]):
                new_cost = cost + (1 if idx<4 else math.sqrt(2))
                
                if new_cost<self.grid_map[s_nxny[0]][s_nxny[1]]:
                    print("chongchongchong")
                    self.grid_map[s_nxny[0]][s_nxny[1]] = new_cost
                    heapq.heappush(pq, (new_cost, nx, ny))
    print("cnt is",cnt)

    return
obstacle_heuristic()
for i in range(len(self.grid_map)):
    print("grid_map",self.grid_map[i])