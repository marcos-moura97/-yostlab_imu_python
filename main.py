from y3SpaceDriver import Y3SpaceDriver

def main():
    imu_driver = Y3SpaceDriver(115200,6)

    imu_driver.run()

if __name__ == '__main__':
    main()
