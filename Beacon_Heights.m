beacon_floor=.75; %height of floor beacons off of the ground

%each of the following are the height of the beacons off the floor,
%defining x-y plane
beacon_2=beacon_floor;
beacon_3=beacon_floor;
beacon_9=beacon_floor;
beacon_100=beacon_floor;

beacons_floor=[beacon_2,beacon_3,beacon_9,beacon_100]; %associated id of beacons on floor

%height in [in] off the floor
beacon_18=85.5+3.5+1+3/8;
beacon_41=86.5+3.5+1+3/8; 


beacons_air=[beacon_18,beacon_41]; %associated id of beacons in air for z-triangulation



in_to_m=0.0254; %converts [in] to [m]
beacon_floor=beacon_floor*in_to_m;

beacons_floor=beacons_floor*in_to_m %gets the height of the floor beacons in [m]


beacons_air=beacons_air*in_to_m %converts heights to [m]
beacons_air_offset=beacons_air-beacon_floor %removes offset from beacons on floor, could be useful in future


