## Kurulum ve Gereksinimler
Kurulumda ekstra bir şey yapılmasına gerek yoktur. Sadece EuclidPlanner global_planner plugininden çağrılmalıdır.
move_base.launch dosyasını güncellemeyi unutmayın.

### 1. Publish Point

RVIZ ekranında Publish Point butonu yardımıyla poligonun köşelerini belirleyen ve 4. nokta seçildikten sonra Float32MultiArray pointleri yayımlayan node.

```
rosrun polygon_publisher polygon_publisher
```
<p>
<img src="full_coverage_path/imgs/point_publisher.png" width="874"/>
</p>

### 2. Projenin Çalıştırılması
```
roslaunch turtlebot3_navigation move_base.launch
```
<p>
<img src="full_coverage_path/imgs/fcpp.png" width="874"/>
</p>

### 3. Sonuçlar
<p>
<img src="full_coverage_path/imgs/ornek1.png" width="374"/>
</p>

<p>
<img src="full_coverage_path/imgs/ornek2.png" width="874"/>
</p>

<p>
<img src="full_coverage_path/imgs/costmap.png" width="874"/>
</p>

#### Engellerin Görselleştirilmesi

<p>
<img src="full_coverage_path/imgs/obstacles.png" width="374" height="300"/>
</p>


#### Hatalı Case
Tarama algoritmasındaki hatalı durum.

<p>
<img src="full_coverage_path/imgs/engelli_hata.png" width="374" height="300"/>
</p>
