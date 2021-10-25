Autonom araba

UART üzerinden start komutu verildiğinde, araç sistemi başlar. bu süre zarfında, püskürtme tankını kontrol eden SPI bağlantısı aracılığıyla ikinci kontrolöre bir sinyal gönderir. araç bir engel tespit edene kadar ilerlemeye devam eder. bu olduğunda, serbest bir yol bulana kadar sola dönecek ve hareketine devam edecektir.
enkoder, aracın hızının izlenmesine yardımcı olmak için motorlara bağlanır.

When the start command is given via the UART, the vehicle system starts. During this time, it sends a signal to the second controller via the SPI link that controls the spray tank. The vehicle continues to move forward until it detects an obstacle. when this happens, it will turn left and continue its movement until it finds a free path.
The encoder is connected to the motors to help monitor the speed of the vehicle.
