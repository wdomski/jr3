/***************************************************************************
 *   Copyright (C) 2015 by Wojciech Domski                                 *
 *   Wojciech.Domski@gmail.com                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "jr3API.h"

tjr3 * jr3;

jr3_force_array scale;
jr3_six_axis_array force;

int main(void)
{
	int err = 0;
	int i = 0, c = 0;
	float tmp[6];

	jr3 = jr3_init(JR3_DEVICE_NAME);

	err = jr3_open(jr3);

	if(err < 0)
	{
		printf("Couldn't open jr3 device\n");
		return -1;
	}



	err = jr3_ioctl(jr3, IOCTL0_JR3_GET_FULL_SCALES, (void *)&scale);

	if( err < 0)
	{
		printf("Problem with obtaining scale from jr3\n");
		return -1;
	}

	do
	{
		err = jr3_ioctl(jr3, IOCTL0_JR3_FILTER0, (void *)&force);

		for( i = 0; i < 3; ++i)
		{
			tmp[i] = (float)force.f[i]*(float)scale.f[i]/16384.0;
		}

		for( i = 0; i < 3; ++i)
		{
			tmp[i+3] = (float)force.m[i]*(float)scale.m[i]/16384.0;
		}

		for( i = 0; i < 6; ++i)
		{
			printf("%5.4f  ", tmp[i]);
		}
		printf("\n");

		/*
		//scale and print data
		printf("%d\n%d\n%d\n%d\n%d\n%d\n",
			force.f[0]*scale.f[0]/16384,
			force.f[1]*scale.f[1]/16384,
			force.f[2]*scale.f[2]/16384,
			force.m[0]*scale.m[0]/16384,
			force.m[1]*scale.m[1]/16384,
			force.m[2]*scale.m[2]/16384);
			*/

		sleep(1);
		//++c;
	}while(c < 10);



	jr3_deinit(jr3);
	return 0;
}




























