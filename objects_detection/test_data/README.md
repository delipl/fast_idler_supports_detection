Correspondence model:
```
cd build/ && make && cd .. && ./build/correspondence_grouping train_foot.pcd  conveyor.pcd -r --descr_rad 3 --algorithm Hough --rf_rad 15 --cg_size 0.1 --cq_thresh 1 
```

ISM:
```
cd implict_shape_model/build && make &&  cd ../../ && ./implict_shape_model/build/implicit_shape_model train_foot.pcd 0 train_foot.pcd 1  conveyor2.pcd 0
```

./implict_shape_model/build/implicit_shape_model ism_train_foot4.pcd 0 ism_train_foot3.pcd 1  ism_test_converoy3.pcd 1 15.0 9.0
ism_test_converoy_slice1.pcd