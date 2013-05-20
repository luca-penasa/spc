#ifndef SVMLIB_HELPER_H
#define SVMLIB_HELPER_H



void writeToSVMlibFile(const vector<vector<vector<float> > > &fields, const std::string filename)
{
    std::ofstream file;
    file.open (filename.c_str());

    std::ostringstream stream;
    stream.imbue (std::locale::classic ());
    stream.precision(4); //lighter files

    int n_points = fields[0].at(0).size(); //number of points
    int n_fields = fields.size(); //n of fields


    for (int i = 0; i < n_points; ++i)
    {
        stream << "0 " ; //a fixed class
        int complete_field_id = 0;
        for (int f= 0; f< n_fields; ++f) //for each field
        {
            int n_multi = fields.at(f).size();
            for (int n = 0 ; n < n_multi; ++n ) //for each component of a field
            {
                float value = fields.at(f).at(n).at(i);
                if (isnan(value))
                {
                    complete_field_id++;
                    continue;
                }
                stream << complete_field_id + 1  << ":" << value << " ";
                complete_field_id++;


            }
        }
        std::string result = stream.str ();
        boost::trim (result);
        stream.str ("");
        file << result << "\n";

    }

    file.close();


}


#endif // SVMLIB_HELPER_H
