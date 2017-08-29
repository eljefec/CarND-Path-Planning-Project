#include "vehicle.h"

std::ostream& operator<<(std::ostream& os, const Vehicle& v)
{
    os << "id:" << v.id << ",vx:" << v.vx << ",vy:" << v.vy << ",s:" << v.s << ",d:" << v.d;
    return os;
}
