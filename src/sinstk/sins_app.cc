//
// Created by Zing Fong on 2022/5/25.
//

#include "sins_app.h"

void SinsApp::SinsMechanizationDemo()
{
    Config config{};  // 配置表
    bool ret = config.ReadConfig("config.ini");
    if (!ret)
        return;
    sins_file_stream_.Init(config);  // 文件流初始化
    
    
    
    
}
