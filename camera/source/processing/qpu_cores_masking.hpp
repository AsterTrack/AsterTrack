/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef QPU_CORE_MASKING_H
#define QPU_CORE_MASKING_H


struct QPUCoreMasking
{
	bool enabled[12] = { 1,1,1,0, 1,1,0,1, 1,0,1,0 };
	// TODO: Some QPUs do not work with this qpu code (bug), disable them
	// Do not waste too much time on this, I already did. These cores don't work with this specific code
	// Feel free to scour https://github.com/Seneral/VC4CV for more info / debugging, not sure there is much
	// As far as I remember, the VPM writes seem to write wrong output on just these cores

	inline int getUsed() const
	{
		int qpuCoresUsed = 0;
		for (int i = 0; i < 12; i++)
			qpuCoresUsed += enabled[i]? 1 : 0;
		return qpuCoresUsed;
	}
};

#endif // QPU_CORE_MASKING_H