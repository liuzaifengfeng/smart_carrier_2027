#pragma once
#include <math.h>
#include <stdint.h>
#include <initializer_list>

namespace field {
struct Rect { float left,right,bottom,top; };
#define FIELD_BOUNDS(size, body) constexpr float SIZE=size, BODY=body;
#define FIELD_OBSTACLE(a,b,c,d)
#include "field_geometry.def"
#undef FIELD_BOUNDS
#undef FIELD_OBSTACLE
#define FIELD_BOUNDS(size,body)
#define FIELD_OBSTACLE(a,b,c,d) {a,b,c,d},
constexpr Rect OBSTACLES[]={
#include "field_geometry.def"
};
#undef FIELD_BOUNDS
#undef FIELD_OBSTACLE
/**
 * @brief 根据场地边界、车体尺寸和障碍坐标生成几何版本签名，供上下位机核对地图一致性。
 */
inline uint32_t signature() {
    uint32_t hash=2166136261u;
    const float bounds[]={SIZE,BODY};
    for(float v:bounds)hash=(hash^static_cast<uint32_t>(lroundf(v*1000)))*16777619u;
    for(const auto &r:OBSTACLES)for(float v:{r.left,r.right,r.bottom,r.top})
        hash=(hash^static_cast<uint32_t>(lroundf(v*1000)))*16777619u;
    return hash;
}
// 返回 nullptr 表示整个300mm方形车体与已知障碍/外边界保持 margin。
// SAT同时检查世界轴与车体轴，不能仅检查四角是否落进矩形。
/**
 * @brief 检查指定位置、航向的整个方形车体与场地边界及障碍是否保持margin。
 * @return nullptr 几何通过，否则为BOUNDARY或OBSTACLE；距离单位mm，航向deg。
 */
inline const char *check(float x,float y,float heading,float margin) {
    const float a=heading*3.14159265358979323846f/180;
    const float fx=cosf(a),fy=sinf(a),rx=sinf(a),ry=-cosf(a),half=BODY/2;
    const float extent=half*(fabsf(fx)+fabsf(rx));
    if(x-extent<margin-0.001f||y-extent<margin-0.001f
        ||x+extent>SIZE-margin+0.001f||y+extent>SIZE-margin+0.001f)return "BOUNDARY";
    for(const Rect &r:OBSTACLES) {
        float dx=x-(r.left+r.right)/2,dy=y-(r.bottom+r.top)/2;
        float hx=(r.right-r.left)/2+margin,hy=(r.top-r.bottom)/2+margin;
        const float axes[][2]={{1,0},{0,1},{fx,fy},{rx,ry}};
        bool separated=false;
        for(const auto &axis:axes) {
            float ux=axis[0],uy=axis[1];
            float body=half*(fabsf(ux*fx+uy*fy)+fabsf(ux*rx+uy*ry));
            float rectangle=hx*fabsf(ux)+hy*fabsf(uy);
            if(fabsf(dx*ux+dy*uy)>=body+rectangle-0.001f){separated=true;break;}
        }
        if(!separated)return "OBSTACLE";
    }
    return nullptr;
}
}
