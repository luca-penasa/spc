//#include <test.h>
//#include <plotter.h>


////#include <cc2DPlotWindow.h>


//#include <ccStructuredReferenceImage.h>



//Test::Test() : BaseFilter(FilterDescription(   "Test",
//                                             "Test",
//                                             "test",
//                                             ":/toolbar/icons/time_series.png") )
//{



//};

//int Test::compute()
//{
////  QDialog * dialog = new QDialog;
////  QVBoxLayout *mainLayout = new QVBoxLayout;

////  QProgressBar * bar = new QProgressBar;
////  cc2DPlotWindow * plotter = new cc2DPlotWindow;
////  plotter->show();
////  dialog->setLayout(mainLayout);
////  mainLayout->addWidget(plotter);
////  dialog->show();


//    //ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();
////    ccStructuredReferenceImage referencer;

////    referencer.setXStep(0.001);
////        referencer.setYStep(0.001);
////        referencer.setZStep(0.001);

////        referencer.setBoundingBoxFromCloud(cloud);

////        referencer.updateAllPixelsCentersFromBB();


////        std::cout << "MAPPING..." << std::endl;
////        CloudToImageMapping map = referencer.computeMappingForCloud(cloud);

////        std::cout << "To Image..." << std::endl;
////        vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

////        image = referencer.mapToImage(map, cloud);

////        vtkSmartPointer<vtkDataSetWriter> w = vtkSmartPointer<vtkDataSetWriter>::New();
////        w->SetInput(image);
////        w->SetFileName("/home/luca/Desktop/tmp.vtk");
////        w->Write();








//  return 1;
//};

//int Test::checkSelected()
//{
//  return 1;
//}
