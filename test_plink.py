from pyplink import Plink, BrakeMode
import time

def main():
    plink = Plink()
    plink.connect()

    plink.channel1.brake_mode = BrakeMode.COAST


if __name__ == "__main__":
    main()