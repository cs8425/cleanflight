/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

extern int32_t AltHold;
extern int32_t vario;

void calculateEstimatedAltitude(timeUs_t currentTimeUs);

struct pidProfile_s;
void configureAltitudeHold(struct pidProfile_s *initialPidProfile);

struct rxConfig_s;
void applyAltHold(struct rxConfig_s *rxConfig);
void updateAltHoldState(void);
void updateSonarAltHoldState(void);

void calculateEstimatedAltitudeACC(timeUs_t currentTime);
void updateACCAltHoldState(void);
void resetACCVel(void);

int32_t altitudeHoldGetEstimatedAltitude(void);
