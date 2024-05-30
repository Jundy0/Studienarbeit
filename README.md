# Studienarbeit

GitHub Repo für unsere Studienarbeit

# Documentation:

- [Datasheet](https://bucket-download.slamtec.com/d1e428e7efbdcd65a8ea111061794fb8d4ccd3a0/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf)
- [User Manual](https://bucket-download.slamtec.com/af084741a46129dfcf2b516110be558561d55767/LM108_SLAMTEC_rplidarkit_usermanual_A1M8_v2.2_en.pdf)
- [SDK User Manual](https://bucket-download.slamtec.com/6957283725b66750890024d1f0d12940fa079e06/LR002_SLAMTEC_rplidar_sdk_v2.0_en.pdf)

## Repositories:

- [SDK](https://github.com/Slamtec/rplidar_sdk)
- [Example](https://github.com/berndporr/rplidar_rpi)

# Setup:

1. Clone this repo

```bash
git clone https://github.com/Jundy0/Studienarbeit --recurse-submodules
```

2. Install pigpio

```bash
sudo apt-get install libpigpio-dev
```

3. Install PCL

```bash
sudo apt install libpcl-dev
```

# Lidar (Running on Raspberry Pi):

## Build:

1. Go to Lidar Folder

```bash
cd src/lidar
```

2. Create Build Folder

```bash
mkdir build
```

3. Go to Build Folder

```bash
cd build
```

4. Gernate Build Files

```bash
cmake .. -G "Unix Makefiles"
```

5. Build the project

```bash
make
```

# Simulation (sfml has to be installed):

## Install sfml:

```bash
sudo apt-get install libsfml-dev
```

## Build:

1. Go to Simulation Folder

```bash
cd src/simulation
```

2. Create Build Folder

```bash
mkdir build
```

3. Go to Build Folder

```bash
cd build
```

4. Gernate Build Files

```bash
cmake .. -G "Unix Makefiles"
```

For Debugging use 

```bash
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
```

5. Build the project

```bash
make
```
