#include "2dplotter.h"

void
TwoDPlotter::addLine(const std::vector<float> &x, 
				const std::vector<float> &y, 
				const std::string &y_name = "y"
)
{
	
	int n_values = x.size();
	//a new table
	vtkSmartPointer<vtkTable> 
	table = vtkSmartPointer<vtkTable>::New();
	
	//an array for x
	vtkSmartPointer<vtkFloatArray> 
	arrX =	vtkSmartPointer<vtkFloatArray>::New();
	
	arrX->SetName("x");
	
	table->AddColumn(arrX);
	
	//now one for y
	vtkSmartPointer<vtkFloatArray> 
	arrY =	vtkSmartPointer<vtkFloatArray>::New();
	
	arrY->SetName(y_name.c_str());
	table->AddColumn(arrY);
	
	
	//Fill the table
	table->SetNumberOfRows(n_values);
	for (int i = 0; i < n_values; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
	}
	
	vtkPlot *line = chart_->AddPlot(vtkChart::LINE);
	line->SetInput(table, 0, 1);
	line->SetColor(0, 0, 255, 255);
	line->SetWidth(4.0);
	// 		vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CROSS);
	
};


void
TwoDPlotter::addScatter(const std::vector<float> &x, 
					 const std::vector<float> &y, 
					 const std::string &y_name = "y"
)
{
	
	int n_values = x.size();
	//a new table
	vtkSmartPointer<vtkTable> 
	table = vtkSmartPointer<vtkTable>::New();
	
	//an array for x
	vtkSmartPointer<vtkFloatArray> 
	arrX =	vtkSmartPointer<vtkFloatArray>::New();
	
	arrX->SetName("x");
	
	table->AddColumn(arrX);
	
	//now one for y
	vtkSmartPointer<vtkFloatArray> 
	arrY =	vtkSmartPointer<vtkFloatArray>::New();
	
	arrY->SetName(y_name.c_str());
	table->AddColumn(arrY);
	
	
	//Fill the table
	table->SetNumberOfRows(n_values);
	for (int i = 0; i < n_values; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
	}
	
	vtkPlot *points = chart_->AddPlot(vtkChart::POINTS);
	
	points->SetInput(table, 0, 1);
	points->SetColor(0, 0, 0, 255);
	points->SetWidth(1.0);
// 	vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CROSS);
	
};