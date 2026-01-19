//
// Created by 13513 on 25-12-30.
//

#ifndef REMOTE_H
#define REMOTE_H

#include "remote_DT7.h"
#include "remote_VT13.h"
#include "remote_FS_I6X.h"

template <class Driver>
class Remote : public Driver
{
public:
    using Driver::Driver; // 继承构造函数
};

#endif //REMOTE_H

