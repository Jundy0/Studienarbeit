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
git clone https://github.com/Jundy0/Studienarbeit
```

2. Go to the folder

```bash
cd Studienarbeit
```

3. Install the sdk

```bash
git submodule update --init --recursive
```

4. Install pigpio

```bash
sudo apt-get install libpigpio-dev
```

# Build:

1. Create Build Folder

```bash
mkdir build
```

2. Go to the folder

```bash
cd build
```

3. Build the project

```bash
cmake .. -G "Unix Makefiles"
```

4. Build the project

```bash
make
```
