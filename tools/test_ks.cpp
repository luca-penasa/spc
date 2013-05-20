#include <kernel_smoothing.cpp>


int main(int argc, char *argv[])
{
	
	float start = 0;
	float end = 100;
	float step_1 = 0.01;
	float step_2 = 0.5;
	
	//construct some data
	
	vType x = subdivideRange(start, end, step_1);
	int n = x.size();
	
	vType y;
	y.resize(n);
	
	for (int i = 0; i < n; ++i)
	{
// 		float random_number = rand()%1000 - 500 / 1000;
		y[i] = x[i] * x[i] + 5; //+ random_number;
	}
	
	
	//Use  NNInterpolator
	InterpolatorNN iNN(x, y);
	
	
// 	KernelSmoothing ks (x, y);
	
	//produce the vector where to evaluate

	
	
// 	idvType ids;
// 	vType dist;
// 	//now try to retrieve some neighbors
// 	int nn = ks.radiusSearch((float) 10, (float) 5, ids, dist);
// 	cout << nn << endl;
// 	
// 	for (int i = 0; i< ids.size(); ++i)
// 	{
// 		cout << ids[i] << endl;
// 	}
// 	
	
	//just try to compute one value
//  	nType ecco = ks.evaluateKS((float) 10, (float)5);
// 	cout << ecco << endl;

	
	vType new_x = subdivideRange(start, end, step_2);
	vType new_y = iNN.compute(new_x);
	
	ofstream myfile;
	myfile.open ("newseries.txt");
	
	
	
	
    for (int i = 0; i < new_x.size(); ++i)
  	{
 		myfile << new_x[i] << " " << new_y[i] << endl ;
  	}
  	
  	myfile.close();
 	
 	
 	ofstream myfile2;
 	myfile2.open ("originalseries.txt");
 	
 	
 	
 	
 	for (int i = 0; i < x.size(); ++i)
 	{
 		myfile2 << x[i] << " " << y[i] << endl ;
 	}
 	
 	myfile2.close();
 	
 	
 
// 	cout << sqrt(0) << endl;
	
	return 1;
}