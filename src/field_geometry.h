// 地图坐标和线路几何的共享定义。
// 节点路线使用同一套几何，修改坐标时应核对路线规划和上位机下发所依据的版本。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <math.h>  // 引入 math.h 中的声明和定义。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <initializer_list>  // 引入 initializer_list 中的声明和定义。

namespace field {  // 定义 field（场地） 的类型成员或作用域。
struct Rect { float left,right,bottom,top; };  // 定义 Rect 的类型成员或作用域。
#define FIELD_BOUNDS(size, body) constexpr float SIZE=size, BODY=body;  // 为后续文本展开定义宏 FIELD_BOUNDS(size, body) constexpr float SIZE=size, BO…。
#define FIELD_OBSTACLE(a,b,c,d)  // 为后续文本展开定义宏 FIELD_OBSTACLE(a,b,c,d)。
#include "field_geometry.def"  // 引入 field_geometry.def 中的声明和定义。
#undef FIELD_BOUNDS  // 撤销宏 FIELD_BOUNDS，限定其作用范围。
#undef FIELD_OBSTACLE  // 撤销宏 FIELD_OBSTACLE，限定其作用范围。
#define FIELD_BOUNDS(size,body)  // 为后续文本展开定义宏 FIELD_BOUNDS(size,body)。
#define FIELD_OBSTACLE(a,b,c,d) {a,b,c,d},  // 为后续文本展开定义宏 FIELD_OBSTACLE(a,b,c,d) {a,b,c,d},。
constexpr Rect OBSTACLES[]={  // 声明 OBSTACLES，类型和可用范围由所在声明决定。
#include "field_geometry.def"  // 引入 field_geometry.def 中的声明和定义。
};  // 结束此初始化列表或局部作用域。
#undef FIELD_BOUNDS  // 撤销宏 FIELD_BOUNDS，限定其作用范围。
#undef FIELD_OBSTACLE  // 撤销宏 FIELD_OBSTACLE，限定其作用范围。
/**
 * @brief 根据场地边界、车体尺寸和障碍坐标生成几何版本签名，供上下位机核对地图一致性。
 */
// 根据几何定义生成签名供配置比对。
// 上位机与固件若使用不同几何，调用端可在准备路径时发现版本差异。
inline uint32_t signature() {  // 定义函数或成员 的入口 signature，参数由此处的签名确定。
    uint32_t hash=2166136261u;  // 将 2166136261u 写入或初始化 hash。
    const float bounds[]={SIZE,BODY};  // 声明 bounds，类型和可用范围由所在声明决定。
    for(float v:bounds)hash=(hash^static_cast<uint32_t>(lroundf(v*1000)))*16777619u;  // 按 float v:bounds)hash=(hash^static_cast<uint32_t>(lround… 组织循环的初值、继续条件和步进。
    for(const auto &r:OBSTACLES)for(float v:{r.left,r.right,r.bottom,r.top})  // 按 const auto &r:OBSTACLES)for(float v: 组织循环的初值、继续条件和步进。
        hash=(hash^static_cast<uint32_t>(lroundf(v*1000)))*16777619u;  // 将 (hash^static_cast<uint32_t>(lroundf(v*1000)))*16777619u 写入或初始化 hash。
    return hash;  // 结束当前函数并返回 hash。
}  // 结束此函数或控制块。
// 返回 nullptr 表示整个300mm方形车体与已知障碍/外边界保持 margin。
// SAT同时检查世界轴与车体轴，不能仅检查四角是否落进矩形。
/**
 * @brief 检查指定位置、航向的整个方形车体与场地边界及障碍是否保持margin。
 * @return nullptr 几何通过，否则为BOUNDARY或OBSTACLE；距离单位mm，航向deg。
 */
inline const char *check(float x,float y,float heading,float margin,bool checkBoundary=true) {  // 定义函数或成员 的入口 check，参数由此处的签名确定。
    const float a=heading*3.14159265358979323846f/180;  // 将 heading*3.14159265358979323846f/180 写入或初始化 a。
    const float fx=cosf(a),fy=sinf(a),rx=sinf(a),ry=-cosf(a),half=BODY/2;  // 将 cosf(a),fy=sinf(a),rx=sinf(a),ry=-cosf(a),half=BODY/2 写入或初始化 fx。
    if(checkBoundary) {  // 当 checkBoundary 成立时进入此分支，否则继续后续流程。
        const float extent=half*(fabsf(fx)+fabsf(rx));  // 将 half*(fabsf(fx)+fabsf(rx)) 写入或初始化 extent。
        if(x-extent<margin-0.001f||y-extent<margin-0.001f  // 当 x-extent<margin-0.001f||y-extent<margin-0.001f 成立时进入此分支，否则继续后续流程。
            ||x+extent>SIZE-margin+0.001f||y+extent>SIZE-margin+0.001f)return "BOUNDARY";  // 继续上一行的组合条件，本行提供 ||x+extent>SIZE-margin+0.001f||y+extent>SIZE-margin+0.001f)return "BOUNDARY";。
    }  // 结束此条件分支。
    for(const Rect &r:OBSTACLES) {  // 按 const Rect &r:OBSTACLES 组织循环的初值、继续条件和步进。
        float dx=x-(r.left+r.right)/2,dy=y-(r.bottom+r.top)/2;  // 将 x-(r.left+r.right)/2,dy=y-(r.bottom+r.top)/2 写入或初始化 dx。
        float hx=(r.right-r.left)/2+margin,hy=(r.top-r.bottom)/2+margin;  // 将 (r.right-r.left)/2+margin,hy=(r.top-r.bottom)/2+margin 写入或初始化 hx。
        const float axes[][2]={{1,0},{0,1},{fx,fy},{rx,ry}};  // 声明 axes，类型和可用范围由所在声明决定。
        bool separated=false;  // 将 false 写入或初始化 separated。
        for(const auto &axis:axes) {  // 按 const auto &axis:axes 组织循环的初值、继续条件和步进。
            float ux=axis[0],uy=axis[1];  // 将 axis[0],uy=axis[1] 写入或初始化 ux。
            float body=half*(fabsf(ux*fx+uy*fy)+fabsf(ux*rx+uy*ry));  // 将 half*(fabsf(ux*fx+uy*fy)+fabsf(ux*rx+uy*ry)) 写入或初始化 body（车体）。
            float rectangle=hx*fabsf(ux)+hy*fabsf(uy);  // 将 hx*fabsf(ux)+hy*fabsf(uy) 写入或初始化 rectangle。
            if(fabsf(dx*ux+dy*uy)>=body+rectangle-0.001f){separated=true;break;}  // 当 fabsf(dx*ux+dy*uy)>=body+rectangle-0.001f 成立时进入此分支，否则继续后续流程。
        }  // 结束此循环体。
        if(!separated)return "OBSTACLE";  // 若 !separated，立即返回 "OBSTACLE"。
    }  // 结束此循环体。
    return nullptr;  // 结束当前函数并返回 nullptr。
}  // 结束此初始化列表或局部作用域。
}  // 结束命名空间。
