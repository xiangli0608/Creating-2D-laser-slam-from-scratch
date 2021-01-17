/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lesson4/hector_mapping/hector_mapping.h"

namespace hector_mapping
{
class HectorMapping
{
private:
    /* data */
public:
    HectorMapping(/* args */);
    ~HectorMapping();
};

HectorMapping::HectorMapping(/* args */)
{
}

HectorMapping::~HectorMapping()
{
}


} // namespace hector_mapping

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_make_hector_map");

    hector_mapping::HectorMapping hector_mapping;

    ros::spin();

    return (0);
}