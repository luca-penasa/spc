#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <spc/methods/common.h>
#include <spc/methods/strings.h>
#include <spc/io/io_helper.h>

//#include <spc/methods/linear_interpolator.h>

#include <math.h>

#include<spc/methods/pcl_helper.h>

using namespace pcl;
using namespace pcl::console;
using namespace std;

enum NORMALIZE_TYPES {NO, STAT, RANGE}; //two types and none




//A strange point type to be used here:
struct PointScalar
    {

        float Sc4laR897; //should be an uncommon name :-|




        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointScalar,
                         (float, Sc4laR897, Sc4laR897)


    )


    struct file
    {
        vector<int> fields_ids_;
        vector<string> fields_names_;
        string path_;
    };



    vector< file >
    readConfigFile2(const string &filename)
    {
        vector<file> files;

        //read config file
        std::ifstream in(filename.c_str());
        if (!in.is_open())
        {
            print_error("Error loading config file - cannot find it?\n");
            return files;
        }

        string line;  //to keep string to be tokenized
        vector<string> vec; //to keep tokenized string

        file this_file;

        while ( in.eof() != 1 )
        {

            std::getline(in, line);

            //void lines could be accientaly present, in case continue
            if (line.size() == 0)
            {
                continue;
            }

            //Tokenize the line
            typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
            boost::char_separator<char> sep(" ");
            tokenizer tokens(line, sep);

            //Assign tokens to a string vector
            vec.clear();
            vec.assign(tokens.begin(), tokens.end());

            if (vec.size() == 1) //we got a file definition
            {
                if (this_file.path_.empty()) //we are at first file definition line
                {
                    this_file.path_ = vec[0]; //put in this_file
                }
                else //we got a new file
                {
                    //push back the old file
                    files.push_back(this_file);
                    this_file.fields_ids_.clear();
                    this_file.fields_names_.clear();
                    //change to the new file
                    this_file.path_ = vec[0];
                }
                continue; //goto next line

            }

            //if more than one we are talking about fields
            int f_id  = atoi(vec.at(0).c_str());
            string f_name = vec.at(1);
            this_file.fields_ids_.push_back(f_id);
            this_file.fields_names_.push_back(f_name);
            //then read another line....

        }
        files.push_back(this_file);

        return files;
    }


template<typename ScalarT>
void
getMeanAndStd(const std::vector<ScalarT> &v, ScalarT &mean, ScalarT &std )
{
    mean = 0.0f;

    BOOST_FOREACH (auto p, v)
        mean += p;

    mean /= v.size();


    std = 0.0f;
    BOOST_FOREACH (auto p, v)
        std += (p - mean) * (p - mean);

    std = sqrt(std/v.size());

    return;
}

template<typename ScalarT>
void
getMinMax(const std::vector<ScalarT> &v, ScalarT &min, ScalarT &max)
{
    min = *min_element(v.begin(), v.end(), [](ScalarT a, ScalarT b){ return ( a < b );} );

    max = *max_element(v.begin(), v.end(), [](ScalarT a, ScalarT b){ return ( a < b );} );
    return;
}

template<typename ScalarT>
void
normalizeVector(std::vector<ScalarT> &v, const NORMALIZE_TYPES kind)
{
    switch (kind)
        {
            case (NO):
            {
                break;
            }
            case (STAT):
            {
                //estimate mean and std of the scalar field
                ScalarT mean, std;
                getMeanAndStd(v, mean, std);
                BOOST_FOREACH (auto &val, v)
                    val = (val - mean) / std;


                break;
            }
            case(RANGE):
            {
                ScalarT min, max;
                getMinMax(v, min, max);
                BOOST_FOREACH (auto &val, v)
                    val = (val - min) / (max - min);

                break;
            }
        }

        return;
}

template<typename ScalarT>
void
normalizeField(std::vector<std::vector<ScalarT> > & field, const NORMALIZE_TYPES kind)
{
    BOOST_FOREACH (auto &f, field)
        normalizeVector(f, kind);

    return;
}

//PointCloud<PointScalar>
//getScaledField(const int id,pcl::PCLPointCloud2::Ptr cloud, string &field_name, const NORMALIZE_TYPES kind)
//{
//    PointCloud<PointScalar> pcl_cloud;
//    field_name = cloud->fields[id].name;
//    (*cloud).fields[id].name = "Sc4laR897";
//    fromPCLPointCloud2(*cloud, pcl_cloud);

//    normalizeField(pcl_cloud, kind );

//    //reset the changed name
//    (*cloud).fields[id].name = field_name;
//    return pcl_cloud;
//}

/// note we are not checking for fields with duplicated names, they will be simply overwritten
pcl::PCLPointCloud2
mergeAll(const vector<pcl::PCLPointCloud2> &clouds)
{

   pcl::PCLPointCloud2 out_cloud;
   pcl::PCLPointCloud2 tmp_cloud;

    int c = 0; //counter
    BOOST_FOREACH (auto cloud, clouds)
    {
        if (c == 0)
        {
            tmp_cloud = cloud;
            c++;
            continue;
        }
        concatenateFields(tmp_cloud, cloud, out_cloud);
        tmp_cloud = out_cloud;
        c++;
    }

    return out_cloud;
}


///////////////////////////// MAIN //////////////////////////////////////////////////////////

int main(int argc, char ** argv)
{

    vector<int> txt_indices = parse_file_extension_argument(argc, argv, ".txt");

    if ((txt_indices.size() == 0) || (txt_indices.size() > 1))
    {
        print_error("Please supply a txt file from which read variables configuration\n");
        return -1;
    }

    vector<int> pcd_indices = parse_file_extension_argument(argc, argv, ".pcd");
    if ((pcd_indices.size() == 0) || (pcd_indices.size() > 1))
    {
        print_error("Please supply a pcd file that will contain out variables\n");
        return -1;
    }

    string norm_type;
    NORMALIZE_TYPES ntype = NO;
    parse_argument(argc, argv, "-n", norm_type);

    if (norm_type == string("stat"))
    {
        ntype = STAT;
    }
    else if (norm_type == string("range"))
    {
        ntype = RANGE;
    }

    vector<file> files = readConfigFile2(argv[txt_indices[0]]);

    print_highlight("Extracting these fields:\n");
    BOOST_FOREACH (auto file, files)
    {
        print_info("Fields to be extracted from cloud %s:\n", file.path_.c_str());
        BOOST_FOREACH (auto field , file.fields_names_)
        {
            print_info("  --> %s\n", field.c_str() );
        }
    }

    //// GET THE FIELDS
   pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
   pcl::PCLPointCloud2::Ptr tmp_cloud (new pcl::PCLPointCloud2);


    vector<pcl::PCLPointCloud2> collected_fields;

    BOOST_FOREACH (auto file, files) // for each file
    {
        print_info("Reading %s\n", file.path_.c_str());
        //load it
        pcl::io::loadPCDFile(file.path_, *cloud);

        BOOST_FOREACH (auto field, file.fields_ids_) //for each field to be taken from this cloud
        {

            string this_field_name = cloud->fields.at(field).name;


            std::vector< std::vector<float> > std_field = spc::readCompleteFieldToVector<float>(*cloud, this_field_name);
            normalizeField(std_field, ntype);
            *tmp_cloud = spc::fromStdVectorToSensor(std_field, this_field_name);
            collected_fields.push_back(*tmp_cloud);



        }





    }




    print_info("Merging everything in a single file\n");
    auto all_fields = mergeAll(collected_fields); //merge all require too much memory, must be rewritten completely

    spc::io::savePCDBinaryCompressed( argv[pcd_indices[0]], all_fields);





    return 1;
}
