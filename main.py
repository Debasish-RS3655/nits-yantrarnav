from model_pipeline import predict_image_class
import joblib


def predict(model, image_path):


    model = joblib.load(model_path)


    predicted_class = predict_image_class(model, image_path)

    return predicted_class

if __name__ == '__main__':
    model_path = 'xgb_best_model.joblib'  
    image_path = 'Dataset/Rocky/0003ML0000001200100140E01_DRCL.JPG' 
    value =predict( model_path , image_path)

    if value:
        print(f"The predicted class for the image is: {value}")
    else:
        print("The image could not be processed.")