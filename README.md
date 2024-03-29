> ### Proje Ekip Arkadaşları :
- github/sukrugorur
- github/dilaraerdogan 

> # **Self Driving Car ( Teknofest Robotaksi )**
Teknofest Robotaksi yarışmasına ait haritayı başarılı şekilde tamamlayan otonom araç algoritması tasarlanıp simülasyon üzerinde test edilerek kodlanmıştır.
Araç üzerinde çalıştırılan algoritma haritayı doğru ve başarılı bir şekilde geçip son olarak park alanlarından uygun olan kısıma park edip görevlerini tamamlamıştır.
Aracımız üzerinde çalışan bir çok sistem bulunmaktadır. Bu sistemler aşağıda belirtilmiştir.
  - Şerit Takip Sistemi
  - Çevre Kontrol Sistemi
  - Trafik Levhaları Tanıma Sistemi
  - PID Kontrol Sistemi
  - Araç Yön Karar Sistemi
  - Otonom Park Sistemi

Tüm bu sistemleri ortak bir üst sistem tarafından kontrol edilerek araç üzerinde çalıştırılmıştır. Aracımız haritadaki şeritleri takip ederek yoluna devam etmektedir. Ayrıca
haritanın belirli yerlerine konumlandırılan trafik işaretlerine uygun şekilde yönlendirilmektedir. Trafik işaretleri ikiye ayrılarak tasarlanmıştır. Bu ayrım _direction signs_
ve _stop signs_ olarak belirlenmiştir. Direction Signs olarak belirlenen levhaların tamamı aracın doğru konuma yönlendirilmesini sağlarken, Stop Signs olarak belirlenen levhalar
araç için duruş/kalkış noktaları olarak kararlaştırılmıştır.

Aracımızın harita üzerinde gerçekleştirdiği otonom sürüş sistemine ait test sürüş videosuna **[Self Driving Car](https://youtu.be/Yh4sCAuJSCA)** bağlantısından ulaşabilirsiniz.

Tasarlanan ve kodlanan otonom sürüş algoritmaları [Webots](https://cyberbotics.com/) simülasyonu üzerinde çalıştırılarak test edilmiştir. Kullanılan simülasyon ortamına ait
harita tasarımı **map.wbt** dosyası olarak paylaşılmıştır. Haritanın genel kuş bakışı görüntüsü aşağıda gösterilmiştir.

  - Simülasyon Ortamı Tam Sürüm : [Webots R2021b](https://github.com/cyberbotics/webots/releases/tag/R2021b)

![Car Camera](./images/map.png)

> ## Şerit Takip Sistemi
Araç üzerine konumlandırılan kameradan gelen görüntüyle gerçekleştirilmiştir. Anlık olarak kameradan gelen görüntü verisi üzerinden şeritler tespit edilerek takip sistemi
geliştirilmiştir. Görüntü verisi öncelikle perspektif çarpıtma yöntemiyle kuş bakışı hale getirilmiştir. Kuş bakışı hale getirilen görüntü verisi bir sonraki aşama olan görüntü
eşikleme yöntemiyle _(thresholding)_, ikili _(binary)_ görüntüye çevrilmiştir. Bu yöntem sayesinde şeritler belirginleştirilmiş ve tespit edilmiştir. Tespiti gerçekleştirilen
şeritlerle araç kamerasının orta noktasına olan uzaklığını hata değeri olarak bulunmasını sağlamıştır. Bu hata değeri araca yön verilmesi için PID kontrol sistemine gönderilerek
şeritlere göre aracın yol üzerinde hareketini sağlamıştır.


![Car Camera](./images/car_camera.jpg)

> ## Çevre Kontrol Sistemi
Aracın şerit takip sistemiyle hareketi sağlanırken oluşabilecek olumsuz durumları ortadan kaldırmak için geliştirilmiştir. Bazı beklenmeyen durumlarda araç kamerasındaki görüntüde
şeritlerin kaybolması durumunda doğru şekilde aracı yönlendirmek amaçlanmıştır. Bu geliştirilen algoritmada hangi ( sağ ve sol ) şeride göre araç yönlendirilmesi yapılıyorsa,
anlık olarak şeridin kaybolması durumunda araç ilgili yöne belirlenen açı ile yönlendirilerek ilgili şeridi tekrar bulması sağlanmıştır. Aynı zamanda şerit takibinin devre dışı kaldığı
durumlarda aracın yol dışına çıkarak çevre engellere çarpması engellenmiştir.

> ## Trafik Levhaları Tanıma Sistemi
Hazırlanan harita üzerinde aracın doğru şekilde görevlerini tamamlayabilmesi ve trafik kurallarına uygun hareket edebilmesi için levha tanıma sistemi geliştirilmiştir.
Levhaların tanınması için **[YOLOV4](https://github.com/AlexeyAB/darknet)** nesne tanıma algoritması kullanılmıştır. Haritada kullanılacak olan trafik levhalarına ait bir çok görsel etiketlenerek
**[YOLOV4](https://github.com/AlexeyAB/darknet)** algoritmasıyla eğitimi gerçekleştirilmiştir. 
Eğitim sonucunda çıkan ağırlık dosyası kaydedilerek kameradan anlık olarak gelen görüntü üzerinde kullanılarak trafik levhalarının tanınması sağlanmıştır.
Eğitim bulut sisteminde **[Google Colab Notebook](https://colab.research.google.com/drive/1_GdoqCJWXsChrOiY8sZMr_zbr_fH-0Fg?usp=sharing)** üzerinden tamamlanmıştır.
- dur, durak, gecit_yok, ileri_saga, ileri_sola, kirmizi_isik, park, park_yasak, 
- saga_donus_yok, saga_yon, sola_donus_yok, sola_yon, yesil_isik

![Trafic Sign Detection](./images/traffic_sign_detection.png)


> ## PID Kontrol Sistemi
Aracımız hareketini şerit takip sistemine göre gerçekleştirmektedir. Şerit takip sisteminde de anlatıldığı gibi, kameradan gelen görüntü verisine göre bir hata değeri hesaplanmaktadır. Bu hata değerini en aza indirecek şekilde araç yönlendirilmesi yapılmaktadır. Araç yönlendirilmesi **[PID](https://tr.wikipedia.org/wiki/PID)** kontrol döngü yöntemiyle yapılmaktadır. Bir PID denetleyicisi sürekli olarak bir hata değerini, yani amaçlanan sistem durumu ile mevcut sistem durumu arasındaki farkı hesaplamaktadır. Denetleyici süreç kontrol girdisini ayarlayarak hatayı en aza indirmeye çalışmaktadır. Bu durumlara uygun ürettiği çıkış değerini aracın yön açı değerine eşitlenmesiyle otonom sürüş sistemi sağlanmıştır. PID kontrol sistemleri üç ayrı sabit parametreye ihtiyaç duymaktadır. Bu değerler **| Kp, Kd, Ki |** olmak üzere, aracımıza uygun şekilde belirlenmiştir.

![PID Controller](http://stm32f4-discovery.net/wp-content/uploads/pid-controller-diagram.png)


> ## Araç Yön Karar Sistemi
Harita üzerinde şerit takip sistemi ile otonom sürüş gerçekleştiren algoritmamız, dönüş veya kavşak bölgelerindeki yön tercihlerini bu sistem sayesinde belirlemiştir. Aracın hangi yöne hareket etmesi gerektiği **[Karar Ağacı](https://tr.wikipedia.org/wiki/Karar_a%C4%9Fac%C4%B1)** algoritmasıyla belirlenmiştir. Harita üzerinde kullanılabilecek tüm yön belirten trafik işaretleri (_direction signs_) belirlenerek uygun kombinasyonları karar ağaç algoritmasının eğitim aşamasında kullanılmıştır. Oluşabilecek mantıklı kombinasyonların kullanılmasıyla algoritmanın **aşırı öğrenmesi** (_overfitting_) sağlanmıştır. Bu şekilde dönüş veya kavşaklarda aracın doğru yöne gitmesi için şerit takip sistemi anlık olarak güncellenerek ilgili şeridin (_sol veya sağ_) takipi sağlanmıştır. Eğitimi yapılan karar ağacının ağırlık dosyası daha sonra kullanılmak üzere kaydedilmiştir. Bu dosya **dt_traffic_last.pkl** olarak paylaşılmıştır.

> ## Otonom Park Sistemi
Haritayı ve görevlerini otonom sürüş algoritmasıyla tamalayan aracımız, son olarak uygun park alanına otonom şekilde park işlemini yapmıştır. Araç harita üzerinde şerit takip sistemi ile hareket etmesine karşı park algoritması devreye girdiğinde, haritanın park alanı kısmında herhangi bir şerit bulunmaması sebebiyle devre dışı bırakılmıştır. Park algoritması devreye girdiği andan itibaren, araç kamerasından gelen görüntü verisi üzerinde trafik işaretlerinden park tabelası tanınmaya çalışılmıştır.
Park tabelalarından herhangi birisi kamera kadrajına girene kadar, kör sürüş diye tabir ettiğimiz sistem devrede olacak şekilde hareketi sağlanmıştır. Bu durumda araç çevre
kontrol sistemiyle beraber belirlenen yönde hareketini sürdürmeye devam etmiştir. Araç kamerasından gelen görüntüde park tabelasının tanınmasından sonra, tabelanın kamera görüntüsündeki piksel derecesinden konumu ile araç kamerasının orta noktası arasındaki fark değeri, PID kontrol sistemine hata değeri olarak yansıtılmıştır. Çıkan bu hata değerine göre araç yönlendirilmesi gerçekleştirilmiştir. Bu hata değeri belirli periyotlarla kontrol edilmesiyle, aracın uygun park alanına parabolik bir hareket ile parkını gerçekleştirmesi sağlanmıştır.

![Autonomous Parking](./images/autonom_parking.png)
