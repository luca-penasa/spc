#ifndef L_TWODPLOTTER_H
#define L_TWODPLOTTER_H


#include <iostream>
#include <vector>


#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkPlotPoints.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>
#include <vtkAxis.h>

class TwoDPlotter
{
	
private:
	//Some vtk objects
	vtkSmartPointer<vtkChartXY> chart_;
	vtkSmartPointer<vtkContextView> view_;
	
	
public:
	
	//Initializer
	TwoDPlotter()
	{
		
		// Set up a 2D scene, add an XY chart to it
		view_ =
		vtkSmartPointer<vtkContextView>::New();
		view_->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
		view_->GetRenderWindow()->SetSize(400, 300);
		
		chart_ =
		vtkSmartPointer<vtkChartXY>::New();
		view_->GetScene()->AddItem(chart_);
		chart_->SetShowLegend(true);
		
	};
	
	void
	addLine(const std::vector<float> &x, 
					const std::vector<float> &y, 
				 const std::string &y_name 
	);
	
	void
	addScatter(const std::vector<float> &x, 
					const std::vector<float> &y, 
				 const std::string &y_name 
	);


	
	
	
	
	
	
	
	void 
	setXLabel(const std::string &x_label)
	{
		chart_->GetAxis(vtkAxis::BOTTOM)->SetTitle(x_label);
	};
	
	void 
	setYLabel(const std::string &y_label)
	{
		chart_->GetAxis(vtkAxis::LEFT)->SetTitle(y_label);
	};
	
// 	void
// 	addLine(const std::vector<float> &x, const std::vector<float> &y)
// 	{
// 	};
	
// 	void 
// 	addDataToTable(const std::vector<float> &x, 
// 								 const std::string &name, 
// 								 vtkSmartPointer<vtkTable> data_table
// 								)
// 	{
// 		vtkSmartPointer<vtkFloatArray> arrX = vtkSmartPointer<vtkFloatArray>::New();
// 		arrX->SetName("X Axis");
// 		table->AddColumn(arrX);
// 	};
	
	void render()
	{
		view_->GetRenderWindow()->SetMultiSamples(0);
		view_->GetInteractor()->Initialize();
		view_->GetInteractor()->Start();
	};
	
};


#endif