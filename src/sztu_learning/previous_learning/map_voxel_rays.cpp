// Offline ray evidence in a 3-D voxel grid. Counts distinct observing frames.
// File layout and projection semantics are documented in build_rviz_map.py.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

template<class T> void read(std::istream& in, T* data, size_t n) {
  if (!in.read(reinterpret_cast<char*>(data), sizeof(T)*n)) throw std::runtime_error("truncated input");
}
int main(int argc, char** argv) {
 try {
  if (argc != 3) throw std::runtime_error("usage: map_voxel_rays input.bin output.bin");
  std::ifstream in(argv[1], std::ios::binary);
  char magic[8]; read(in, magic, 8);
  if (std::string(magic,8) != "SZVOX001") throw std::runtime_error("bad format");
  std::array<uint32_t,3> dims; read(in,dims.data(),3);
  std::array<double,3> lo,res; read(in,lo.data(),3); read(in,res.data(),3);
  uint32_t frames; read(in,&frames,1);
  uint64_t size=uint64_t(dims[0])*dims[1]*dims[2];
  if (!size || size>100000000 || !frames || frames>65534) throw std::runtime_error("invalid extent");
  for(int k=0;k<3;++k) if(!(res[k]>0) || !std::isfinite(lo[k]) || !std::isfinite(res[k])) throw std::runtime_error("bad grid");
  std::vector<uint16_t> hits(size,0), frees(size,0);
  std::vector<int32_t> hit_frame(size,-1), free_frame(size,-1);
  auto index=[&](const std::array<int,3>& p) {return (uint64_t(p[2])*dims[1]+p[1])*dims[0]+p[0];};
  auto inside=[&](const std::array<int,3>& p) {for(int k=0;k<3;++k) if(p[k]<0 || p[k]>=int(dims[k])) return false; return true;};
  uint64_t rays=0;
  for(uint32_t f=0;f<frames;++f) {
   std::array<double,3> a; read(in,a.data(),3);
   uint32_t count; read(in,&count,1);
   if(count>1000000) throw std::runtime_error("bad frame size");
   std::vector<float> points(size_t(count)*3); read(in,points.data(),points.size());
   for(uint32_t p=0;p<count;++p) {
    std::array<double,3> d,b;
    std::array<int,3> endpoint;
    for(int k=0;k<3;++k) {
     b[k]=points[p*3+k];d[k]=b[k]-a[k];
     if(!std::isfinite(a[k]) || !std::isfinite(b[k])) throw std::runtime_error("nonfinite ray");
     endpoint[k]=int(std::floor((b[k]-lo[k])/res[k]));
    }
    bool endpoint_in=inside(endpoint);
    if(endpoint_in) {
     auto i=index(endpoint);
     if(hit_frame[i]!=int(f)) {++hits[i];hit_frame[i]=f;}
    }
    // Clip the segment to the 3-D grid; a ceiling or floor return may clear
    // only those voxels it actually traverses inside the selected height band.
    double enter=0, leave=1;
    for(int k=0;k<3;++k) {
     double high=lo[k]+dims[k]*res[k];
     if(std::abs(d[k])<1e-14) {if(a[k]<lo[k] || a[k]>=high) {enter=1;leave=0;break;}}
     else {
      double t0=(lo[k]-a[k])/d[k],t1=(high-a[k])/d[k];
      if(t0>t1) std::swap(t0,t1);
      enter=std::max(enter,t0);leave=std::min(leave,t1);
     }
    }
    if(leave-enter<=1e-12) continue;
    std::array<int,3> cell,step;
    std::array<double,3> next,delta;
    double start=enter+std::min(1e-9,(leave-enter)*.25);
    for(int k=0;k<3;++k) {
     cell[k]=int(std::floor((a[k]+start*d[k]-lo[k])/res[k]));
     step[k]=(d[k]>0)-(d[k]<0);
     if(step[k]) {
      double boundary=lo[k]+(cell[k]+(step[k]>0))*res[k];
      next[k]=(boundary-a[k])/d[k];delta[k]=res[k]/std::abs(d[k]);
     } else next[k]=delta[k]=std::numeric_limits<double>::infinity();
    }
    while(inside(cell)) {
     if(endpoint_in && cell==endpoint) break; // never clear its own return voxel
     auto i=index(cell);
     if(free_frame[i]!=int(f)) {++frees[i];free_frame[i]=f;}
     double t=std::min({next[0],next[1],next[2]});
     if(t>=leave-1e-12) break;
     for(int k=0;k<3;++k) if(next[k]<=t+1e-12) {cell[k]+=step[k];next[k]+=delta[k];}
    }
    ++rays;
   }
   if(f%50==0) std::cerr<<"frames "<<f+1<<"/"<<frames<<"\n";
  }
  char extra; if(in.get(extra)) throw std::runtime_error("trailing input");
  std::ofstream out(argv[2],std::ios::binary);
  out.write(reinterpret_cast<const char*>(hits.data()),hits.size()*sizeof(uint16_t));
  out.write(reinterpret_cast<const char*>(frees.data()),frees.size()*sizeof(uint16_t));
  if(!out) throw std::runtime_error("write failed");
  std::cout<<"{\"frames\":"<<frames<<",\"rays\":"<<rays<<",\"voxels\":"<<size<<"}\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
}
