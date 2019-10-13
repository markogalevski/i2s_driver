# i2s_driver_stm32f4xx - a very specific driver for a very specific use

This is a driver I'm writing for my project, the Sitcom Machine, for which I will start a repository and Github Pages page once there is anything worth showing.

This driver currently only has interrupt style transmission implemented, as that is what I was planning to use.

A DMA implementation is next, with a blocking implementation last to come.

All communication is in master mode, as the Sitcom Machine merely pushes data via i2s to the audio codec on the STM32F411E Discovery.

Addng tests via CppuTest could also be interesting.
