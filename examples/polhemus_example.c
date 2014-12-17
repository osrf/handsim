/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <stdio.h>
#include <unistd.h>

#include "polhemus_driver/polhemus_driver.h"

int
main(void)
{
  polhemus_conn_t* conn;

  if(!(conn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                   LIBERTY_HS_PRODUCT_ID,
                                   LIBERTY_HS_WRITE_ENDPOINT,
                                   LIBERTY_HS_READ_ENDPOINT)))
  {
    fprintf(stderr, "Failed to connect\n");
    return -1;
  }

  if(polhemus_init_comm(conn, 10))
  {
    fprintf(stderr, "Failed to initialize comms\n");
    return -1;
  }

  double x, y, z, roll, pitch, yaw;
  for(;;)
  {
    polhemus_get_pose(conn, &x, &y, &z, &roll, &pitch, &yaw, 10);
    printf("%lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
  }

  polhemus_disconnect_usb(conn);
  return 0;
}
