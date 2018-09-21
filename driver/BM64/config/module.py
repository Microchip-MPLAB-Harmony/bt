def loadModule():
    component = Module.CreateComponent("bluetooth_bm64", "BM64", "/Bluetooth/Driver/", "config/bm64.py")
    component.setDisplayType("Driver")

    component.addDependency("I2S driver", "DRV_I2S", False)

    component.addDependency("USART PLIB", "UART", False)

    component.addCapability("Bluetooth", "Bluetooth", False)
