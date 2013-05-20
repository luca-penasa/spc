    #include <QtGui/QApplication>
    
    #include <pcl/visualization/pcl_visualizer.h>
    
    #include <QVTKWidget.h>
    
    int main(int argc, char** argv)
		{
			QApplication app(argc, argv);
			QVTKWidget widget;
			pcl::visualization::PCLVisualizer pviz ("", false);
			
			widget.SetRenderWindow(pviz.getRenderWindow());
			
			pviz.setupInteractor(widget.GetInteractor(), widget.GetRenderWindow());
			pviz.getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
			
			pviz.addCoordinateSystem();
			
			widget.show();
			
			app.exec();
			
			return EXIT_SUCCESS;
		}
		